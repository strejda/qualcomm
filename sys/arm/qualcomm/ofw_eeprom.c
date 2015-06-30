/*-
 * Copyright (c) 2006 Warner Losh.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");
/*
 * Generic IIC eeprom support, modeled after the AT24C family of products.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/resource.h>
#include <sys/sx.h>
#include <sys/uio.h>
#include <machine/bus.h>
#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "iicbus_if.h"

#define	IIC_M_WR	0	/* write operation */
#define	MAX_RD_SZ	256	/* Largest read size we support */
#define MAX_WR_SZ	256	/* Largest write size we support */

struct ofw_eeprom_softc {
	device_t	sc_dev;		/* Myself */
	struct sx	sc_lock;	/* basically a perimeter lock */
	struct cdev	*cdev;		/* user interface */
	int		addr;
	int		size;		/* How big am I? */
	int		type;		/* What type 8 or 16 bit? */
	int		rd_sz;		/* What's the read page size */
	int		wr_sz;		/* What's the write page size */
};

#define OFW_EEPROM_LOCK(_sc)		sx_xlock(&(_sc)->sc_lock)
#define	OFW_EEPROM_UNLOCK(_sc)	sx_xunlock(&(_sc)->sc_lock)
#define OFW_EEPROM_LOCK_INIT(_sc)	sx_init(&_sc->sc_lock, "ofw_eeprom")
#define OFW_EEPROM_LOCK_DESTROY(_sc)	sx_destroy(&_sc->sc_lock);
#define OFW_EEPROM_ASSERT_LOCKED(_sc)	sx_assert(&_sc->sc_lock, SA_XLOCKED);
#define OFW_EEPROM_ASSERT_UNLOCKED(_sc) sx_assert(&_sc->sc_lock, SA_UNLOCKED);
#define CDEV2SOFTC(dev)		((dev)->si_drv1)

struct eeprom_desc {
	int	type;
	int	size;
	int	rd_sz;
	int	wr_sz;
};
static struct eeprom_desc at24c128_desc = {
	16, 16 * 1024, 64, 64
};

static struct ofw_compat_data compat_data[] = {
	{"atmel,24c128",		(intptr_t)&at24c128_desc},
	{NULL,				0},
};

/* cdev routines */
static d_open_t ofw_eeprom_open;
static d_close_t ofw_eeprom_close;
static d_read_t ofw_eeprom_read;
static d_write_t ofw_eeprom_write;

static struct cdevsw ofw_eeprom_cdevsw =
{
	.d_version = D_VERSION,
	.d_flags = D_TRACKCLOSE,
	.d_open = ofw_eeprom_open,
	.d_close = ofw_eeprom_close,
	.d_read = ofw_eeprom_read,
	.d_write = ofw_eeprom_write
};



static int
ofw_eeprom_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{

    	return (0);
}

static int
ofw_eeprom_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{

	return (0);
}

static int
ofw_eeprom_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct ofw_eeprom_softc *sc;
	uint8_t addr[2];
	uint8_t data[MAX_RD_SZ];
	int error, i, len, slave;
	struct iic_msg msgs[2] = {
	     { 0, IIC_M_WR, 1, addr },
	     { 0, IIC_M_RD, 0, data },
	};

	sc = CDEV2SOFTC(dev);
//printf("%s: uio_offset: 0x%08llX,  sc->size: 0x%08X\n", __func__, uio->uio_offset, sc->size);
	if (uio->uio_offset == sc->size)
		return (0);
	if (uio->uio_offset > sc->size)
		return (EIO);
	if (sc->type != 8 && sc->type != 16)
		return (EINVAL);
	OFW_EEPROM_LOCK(sc);
	slave = error = 0;
	while (uio->uio_resid > 0) {
		if (uio->uio_offset >= sc->size)
			break;
		len = MIN(sc->rd_sz - (uio->uio_offset & (sc->rd_sz - 1)),
		    uio->uio_resid);
		switch (sc->type) {
		case 8:
			slave = (uio->uio_offset >> 7) | sc->addr;
			msgs[0].len = 1;
			msgs[1].len = len;
			addr[0] = uio->uio_offset & 0xff;
			break;
		case 16:
			slave = sc->addr | (uio->uio_offset >> 15);
			msgs[0].len = 2;
			msgs[1].len = len;
			addr[0] = (uio->uio_offset >> 8) & 0xff;
			addr[1] = uio->uio_offset & 0xff;
			break;
		}
		for (i = 0; i < 2; i++)
			msgs[i].slave = slave;
		error = iicbus_transfer(sc->sc_dev, msgs, 2);
		if (error)
			break;
		error = uiomove(data, len, uio);
		if (error)
			break;
	}
	OFW_EEPROM_UNLOCK(sc);
	return (error);
}

/*
 * Write to the part.  We use three transfers here since we're actually
 * doing a write followed by a read to make sure that the write finished.
 * It is easier to encode the dummy read here than to break things up
 * into smaller chunks...
 */
static int
ofw_eeprom_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct ofw_eeprom_softc *sc;
	int error, len, slave, waitlimit;
	uint8_t data[MAX_WR_SZ + 2];
	struct iic_msg wr[1] = {
	     { 0, IIC_M_WR, 0, data },
	};
	struct iic_msg rd[1] = {
	     { 0, IIC_M_RD, 1, data },
	};

	sc = CDEV2SOFTC(dev);
	if (uio->uio_offset >= sc->size)
		return (EIO);
	if (sc->type != 8 && sc->type != 16)
		return (EINVAL);
	OFW_EEPROM_LOCK(sc);
	slave = error = 0;
	while (uio->uio_resid > 0) {
		if (uio->uio_offset >= sc->size)
			break;
		len = MIN(sc->wr_sz - (uio->uio_offset & (sc->wr_sz - 1)),
		    uio->uio_resid);
		switch (sc->type) {
		case 8:
			slave = (uio->uio_offset >> 7) | sc->addr;
			wr[0].len = 1 + len;
			data[0] = uio->uio_offset & 0xff;
			break;
		case 16:
			slave = sc->addr | (uio->uio_offset >> 15);
			wr[0].len = 2 + len;
			data[0] = (uio->uio_offset >> 8) & 0xff;
			data[1] = uio->uio_offset & 0xff;
			break;
		}
		wr[0].slave = slave;
		error = uiomove(data + sc->type / 8, len, uio);
		if (error)
			break;
		error = iicbus_transfer(sc->sc_dev, wr, 1);
		if (error)
			break;
		// Now wait for the write to be done by trying to read
		// the part.
		waitlimit = 10000;
		rd[0].slave = slave;
		do
		{
		    error = iicbus_transfer(sc->sc_dev, rd, 1);
		} while (waitlimit-- > 0 && error != 0);
		if (error) {
		    printf("waiting for write failed %d\n", error);
		    break;
		}
	}
	OFW_EEPROM_UNLOCK(sc);
	return error;
}

static int
ofw_eeprom_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);
	device_set_desc(dev, "OFW I2C EEPROM");
	return (BUS_PROBE_DEFAULT);
}

static int
ofw_eeprom_attach(device_t dev)
{
	struct ofw_eeprom_softc *sc = device_get_softc(dev);
	const char *dname;
	int dunit, err;
	struct eeprom_desc *desc;

	sc->sc_dev = dev;
	sc->addr = iicbus_get_addr(dev);
	err = 0;
	dname = device_get_name(dev);
	dunit = device_get_unit(dev);
	desc = (struct eeprom_desc *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	sc->size = desc->size;
	sc->type = desc->type;
	sc->rd_sz = desc->rd_sz;
	sc->wr_sz = desc->wr_sz;
	if (sc->rd_sz > MAX_RD_SZ)
		sc->rd_sz = MAX_RD_SZ;
	if (bootverbose)
		device_printf(dev, "size: %d bytes bus_width: %d-bits\n",
		    sc->size, sc->type);
	sc->cdev = make_dev(&ofw_eeprom_cdevsw, device_get_unit(dev), UID_ROOT,
	    GID_WHEEL, 0600, "eeprom%d", device_get_unit(dev));
	if (sc->cdev == NULL) {
		err = ENOMEM;
		goto out;
	}
	sc->cdev->si_drv1 = sc;
	OFW_EEPROM_LOCK_INIT(sc);
out:
	return (err);
}

static device_method_t ofw_eeprom_methods[] = {
	DEVMETHOD(device_probe,		ofw_eeprom_probe),
	DEVMETHOD(device_attach,	ofw_eeprom_attach),

	DEVMETHOD_END
};

static driver_t ofw_eeprom_driver = {
	"ofw_eeprom",
	ofw_eeprom_methods,
	sizeof(struct ofw_eeprom_softc),
};
static devclass_t ofw_eeprom_devclass;

DRIVER_MODULE(ofw_eeprom, iicbus, ofw_eeprom_driver, ofw_eeprom_devclass, 0, 0);
MODULE_VERSION(ofw_eeprom, 1);
MODULE_DEPEND(ofw_eeprom, iicbus, 1, 1, 1);

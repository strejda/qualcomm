/*-
 * Copyright (c) 2015 Michal Meloun
 * All rights reserved.
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
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/smp.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/cpu-v6.h>
#include <machine/cpuinfo.h>
#include <machine/smp.h>
#include <machine/fdt.h>
#include <machine/intr.h>

#include <arm/qualcomm/qcom_scm.h>

#define SCM_INTERRUPTED			 1

#define SCM_BOOT_ADDR			0x1
#define SCM_SVC_BOOT			0x1

#define SCM_FLAG_COLDBOOT_CPU1		0x01
#define SCM_FLAG_COLDBOOT_CPU2		0x08
#define SCM_FLAG_COLDBOOT_CPU3		0x20
#define SCM_FLAG_WARMBOOT_CPU0		0x04
#define SCM_FLAG_WARMBOOT_CPU1		0x02
#define SCM_FLAG_WARMBOOT_CPU2		0x10
#define SCM_FLAG_WARMBOOT_CPU3		0x40

static int cold_boot_flags[] = {
	0,
	SCM_FLAG_COLDBOOT_CPU1,
	SCM_FLAG_COLDBOOT_CPU2,
	SCM_FLAG_COLDBOOT_CPU3,
};

/*
 * Qualcomm Secure Channel Manager
 */

struct scm_cmd {
	/* Command header */
	uint32_t total_len; 		/* total length of command buffer */
	uint32_t cmd_offset;		/* offset of cmd data, from cmd start */
	uint32_t rsp_hdr_offset;	/* offset of response header
					   from cmd start */
	uint32_t cmd_code;		/* packed service and command code */

	intptr_t orig_addr; 		/* original memory address,
					   not a part of protocol */
	/* Response header */
	uint32_t rsp_len;		/* total size of response buffer */
	uint32_t rsp_offset;		/* offset of response data,
					   from start of response header */
	uint32_t rsp_complete;		/* if response is valid */
};

static struct scm_cmd *
build_cmd(int cmd_buf_size, int rsp_buf_size)
{
	struct scm_cmd *cmd;
	int total_size;
	intptr_t buf;

	/* Get total buffer size, use rounded up sizes of all components */
	total_size = roundup2(sizeof(*cmd), 4) +  roundup2(cmd_buf_size, 4) +
	    roundup2(rsp_buf_size, 4);

	/* Buffer must be cacheline aligned (start and end) */
	buf = (intptr_t)malloc(total_size +  2 * cpuinfo.dcache_line_size,
	    M_DEVBUF, M_WAITOK | M_ZERO);

	/* Fill all offsets and sizes */
	cmd = (struct scm_cmd *)roundup2(buf, cpuinfo.dcache_line_size);
	cmd->orig_addr = buf;
	cmd->total_len = total_size;
	cmd->rsp_hdr_offset = offsetof(struct scm_cmd, rsp_len);
	cmd->rsp_len = rsp_buf_size;
	cmd->cmd_offset = roundup2(sizeof(*cmd), 4);

	cmd->rsp_offset = cmd->cmd_offset + roundup2(cmd_buf_size, 4) -
	    cmd->rsp_hdr_offset;
	return (cmd);
}

/* Buffers helpers */
static inline void *
cmd_buf_ptr(struct scm_cmd *cmd)
{
	return ((void *)((intptr_t)cmd + cmd->cmd_offset));
}

static inline void *
rsp_buf_ptr(struct scm_cmd *cmd)
{
	return ((void *)((intptr_t)cmd + cmd->rsp_offset +
	    cmd->rsp_hdr_offset));
}
int qcom_smc(uint32_t type, uint32_t *context_id, vm_offset_t cmd_paddr);

/* Issue one scm command */
static int
scm_cmd(uint32_t svc_id, uint32_t cmd_id, void *cmd_buf, int cmd_len,
    void *resp_buf, int resp_len)
{
	int ret;
	struct scm_cmd *cmd;
	vm_offset_t cmd_paddr;
	uint32_t context_id;

	cmd = build_cmd(cmd_len, resp_len);
	cmd->cmd_code = (svc_id << 10) | cmd_id;

	if (cmd_buf != NULL)
		memcpy(cmd_buf_ptr(cmd), cmd_buf, cmd_len);

	/* Write back whole command structure */
	cmd_paddr = pmap_kextract((vm_offset_t)cmd);
	dcache_wbinv_poc((vm_offset_t) cmd, cmd_paddr, cmd->total_len);

	/* Issue command */
	do {
		ret = qcom_smc(1, &context_id, cmd_paddr);
	} while (ret == SCM_INTERRUPTED);

	if (ret != 0)
		goto fail;

	/* Wait until command is finished */
	do {
		/* invalidate whole command structure */
		dcache_inv_poc((vm_offset_t) cmd, cmd_paddr, cmd->total_len);
	} while (cmd->rsp_complete == 0);

	/* Copy out response */
	if (resp_buf != NULL)
		memcpy(resp_buf, rsp_buf_ptr(cmd), resp_len);
fail:
	free((void *)cmd->orig_addr,  M_DEVBUF);
	return (ret);
}

int
scm_cold_boot_addr(uint32_t addr, int cpu)
{
	int err;
	struct {
		uint32_t flags;
		uint32_t addr;
	} cmd;

	cmd.addr = addr;
	cmd.flags = cold_boot_flags[cpu];
	err = scm_cmd(SCM_SVC_BOOT, SCM_BOOT_ADDR, &cmd, sizeof(cmd), NULL, 0);

	return (err);
}

# $FreeBSD$

TESTSRC=	${SRCTOP}/contrib/netbsd-tests/kernel

TESTSDIR=	${TESTSBASE}/sys/kern

ATF_TESTS_C+=	kern_copyin
ATF_TESTS_C+=	kern_descrip_test
ATF_TESTS_C+=	ptrace_test
ATF_TESTS_C+=	unix_seqpacket_test
TEST_METADATA.unix_seqpacket_test+=	timeout="15"

LDADD.ptrace_test+=			-lpthread
LDADD.unix_seqpacket_test+=		-lpthread

NETBSD_ATF_TESTS_C+=	lockf_test

WARNS?=	5

TESTS_SUBDIRS+=	acct
TESTS_SUBDIRS+=	execve
TESTS_SUBDIRS+=	pipe

.include <netbsd-tests.test.mk>

.include <bsd.test.mk>

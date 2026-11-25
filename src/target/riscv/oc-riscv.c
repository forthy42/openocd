/* -*-mode:c; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*- */
// SPDX-License-Identifier: GPL-2.0-or-later

/*
*   OpenOCD One Cent RISCV target driver
*   Copyright (C) 2026  Bernd Paysan
*   bernd(at)net2o.de
*
* based losely on
*
*   OpenOCD STM8 target driver
*   Copyright (C) 2017  Ake Rehnman
*   ake.rehnman(at)gmail.com
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <termios.h>
#include <helper/log.h>
#include <target/target.h>
#include <target/target_type.h>
#include <target/register.h>
#include <target/breakpoints.h>

static struct reg_cache *oc_riscv_build_reg_cache(struct target *target);
static int oc_riscv_read_core_reg(struct target *target, unsigned int num);
static int oc_riscv_write_core_reg(struct target *target, unsigned int num);
static int oc_riscv_save_context(struct target *target);
static int oc_riscv_restore_context(struct target *target);
static int oc_riscv_adapter_speed(struct target *target, int speed);
static struct oc_riscv_common *target_to_oc_riscv(struct target *target);

const char* regno_group = "general";
static struct reg_feature feature_cpu = {
  .name = "org.gnu.gdb.riscv.cpu"
};
const int regno_size = 32;
const int regno_type = REG_TYPE_UINT32;

static const struct {
	unsigned int id;
	const char *name;
	int flag;
} oc_riscv_regs[] = {
        {  0,  "zero", 0 },
        {  1,  "ra", 0 },
        {  2,  "sp", 0 },
        {  3,  "gp", 0 },
        {  4,  "tp", 0 },
        {  5,  "t0", 0 },
        {  6,  "t1", 0 },
        {  7,  "t2", 0 },
        {  8,  "fp", 0 },
        {  9,  "s1", 0 },
        {  10,  "a0", 0 },
        {  11,  "a1", 0 },
        {  12,  "a2", 0 },
        {  13,  "a3", 0 },
        {  14,  "a4", 0 },
        {  15,  "a5", 0 },
        {  16,  "a6", 0 },
        {  17,  "a7", 0 },
        {  18,  "s2", 0 },
        {  19,  "s3", 0 },
        {  20,  "s4", 0 },
        {  21,  "s5", 0 },
        {  22,  "s6", 0 },
        {  23,  "s7", 0 },
        {  24,  "s8", 0 },
        {  25,  "s9", 0 },
        {  26,  "s10", 0 },
        {  27,  "s11", 0 },
        {  28,  "t3", 0 },
        {  29,  "t4", 0 },
        {  30,  "t5", 0 },
        {  31,  "t6", 0 },
	{  32,  "pc", 0 },
};

#define OC_RISCV_COMMON_MAGIC	0x53544D38U

#define OC_RISCV_NUM_REGS ARRAY_SIZE(oc_riscv_regs)
#define OC_RISCV_PC 32

enum mem_type {
	RAM,
	FLASH
};

struct oc_riscv_algorithm {
	int common_magic;
};

struct oc_riscv_core_reg {
	uint32_t num;
	struct target *target;
};

#define WRITEOUTSIZE 0x1000

struct oc_riscv_common {
	unsigned int common_magic;

	void *arch_info;
	struct reg_cache *core_cache;
	uint32_t core_regs[OC_RISCV_NUM_REGS];

	bool uart_configured;
	uint32_t uart_id;
	uint32_t uart_speed;
	FILE * uart;
	char dbgbuf[WRITEOUTSIZE];
	int dbgindex;

	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, unsigned int num);
	int (*write_core_reg)(struct target *target, unsigned int num);
};

#define JMPNOP 0x6F
#define READREG_INST(i) (((i)==OC_RISCV_PC) ? JMPNOP : (0x13 | (i) << 15))
#define JMP(i, offset) (0x67 | (i) << 15 | (offset) << 20)
#define READOUT_HI(i, data) (0x37 | i << 7 | ((data + 0x800) & 0xFFFFF000))
#define READOUT_LO(i, data) (0x13 | i << 7 | i << 15 | (data & 0xFFF) << 20)
#define READOUT_LO_ZERO(i, data) (0x13 | i << 7 | (data & 0xFFF) << 20)
// Read to register x1
#define READOPB(offset) (0x8003 | offset << 20)
#define READOPW(offset) (0x9003 | offset << 20)
#define READOPL(offset) (0xA003 | offset << 20)
#define JMP_IMM(offset) (((offset & 0x100000) << 11) |	\
			 (offset & 0x0FF000) |		\
			 ((offset & 0x000800) << 9) |	\
			 ((offset & 0x0007FE) << 20))
// write register x2 to offset(x1)
#define WRITEOPB(offset) (0x00110023 | (offset & 0x1F) << 7 | (offset & 0xFE0) << 20)
#define WRITEOPW(offset) (0x00111023 | (offset & 0x1F) << 7 | (offset & 0xFE0) << 20)
#define WRITEOPL(offset) (0x00112023 | (offset & 0x1F) << 7 | (offset & 0xFE0) << 20)

#define TERMSPEED 4000000

static int oc_riscv_adapter_speed(struct target *target, int speed)
{
	struct termios tbuf;
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);

	oc_riscv->uart_speed = speed;
	cfmakeraw(&tbuf);
	tbuf.c_iflag |= CLOCAL;
	cfsetispeed(&tbuf, speed);
	cfsetospeed(&tbuf, speed);
	int retval = tcsetattr(fileno(oc_riscv->uart), 0, &tbuf);
	LOG_DEBUG("Set speed to %d, returned %d", speed, retval);
	if(!retval)
		return ERROR_OK;
	else
		return ERROR_TARGET_FAILURE;
}
static void oc_riscv_open_target(struct target *target)
{
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);
	char uart_filename[64];
	
	snprintf(uart_filename, sizeof(uart_filename),
		 "/dev/ttyUSB%d", oc_riscv->uart_id);
	oc_riscv->dbgindex = 0;
	oc_riscv->uart = fopen(uart_filename, "r+");
	LOG_DEBUG("Open UART %s=%d", uart_filename, fileno(oc_riscv->uart));
	oc_riscv_adapter_speed(target, TERMSPEED);
	oc_riscv->uart_configured = 1;
}
static void oc_riscv_write_byte(struct target *target, uint8_t byte)
{
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);
	if(oc_riscv->dbgindex < WRITEOUTSIZE)
		oc_riscv->dbgbuf[oc_riscv->dbgindex++] = byte;
	else
		LOG_ERROR("dbgindex overflow");
}
static void oc_riscv_writeout(struct target *target)
{
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);
	fwrite(oc_riscv->dbgbuf, oc_riscv->dbgindex, 1, oc_riscv->uart);
	oc_riscv->dbgindex=0;
}
static uint32_t oc_riscv_readin(struct target * target, uint8_t* buf, size_t count)
{
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);
	return fread(buf, count, 1, oc_riscv->uart);
}
static void oc_riscv_write_inst(struct target *target, uint32_t inst)
{
	oc_riscv_write_byte(target, inst);
	inst >>= 8;
	oc_riscv_write_byte(target, inst);
	inst >>= 8;
	oc_riscv_write_byte(target, inst);
	inst >>= 8;
	oc_riscv_write_byte(target, inst);
}
static void oc_riscv_write_Einst(struct target *target, uint32_t inst)
{
	LOG_DEBUG("E inst %08x", inst);
	oc_riscv_write_byte(target, 'E');
	oc_riscv_write_inst(target, inst);
}
static void oc_riscv_write_Xinst(struct target *target, uint32_t inst)
{
	LOG_DEBUG("X inst %08x", inst);
	oc_riscv_write_byte(target, 'X');
	oc_riscv_write_inst(target, inst);
}
static uint32_t oc_riscv_read_pc(struct target *target)
{
	uint32_t tmpbuf;
	oc_riscv_write_Xinst(target, JMPNOP);
	oc_riscv_writeout(target);
	oc_riscv_readin(target, (uint8_t*)&tmpbuf, 4);
	return tmpbuf;
}
static void oc_riscv_gen_regs(struct target *target)
{
	int i;
	for(i=0; i<32; i++)
		oc_riscv_write_Xinst(target, READREG_INST(i));
	oc_riscv_write_Xinst(target, JMPNOP);
}
static void oc_riscv_write_reg(struct target *target, uint32_t i, uint32_t data)
{
	if(((data + 0x800) & 0xFFFFF000) != 0) {
		oc_riscv_write_Einst(target, READOUT_HI(i, data));
		if((data & 0xFFF) != 0)
			oc_riscv_write_Einst(target, READOUT_LO(i, data));
	} else {
		oc_riscv_write_Einst(target, READOUT_LO_ZERO(i, data));
	}
}
static void oc_riscv_gen_write_pc(struct target *target, uint32_t pc, uint32_t data)
{
	if(((data-pc) + 0x100000) < 0x200000) {
		oc_riscv_write_Einst(target, JMP_IMM((data-pc)) | JMPNOP);
	} else {
		// globbers x1
		oc_riscv_write_Einst(target, READOUT_HI(1, data));
		oc_riscv_write_Einst(target, JMP(1, (data & 0xFFF)));
	}
}
static void oc_riscv_gen_reg_writes(struct target *target, uint32_t pc, uint32_t regs[])
{
	int i;
	// write the PC first, may globber x1
	oc_riscv_gen_write_pc(target, pc, regs[OC_RISCV_PC]);
	for(i=0; i<OC_RISCV_PC; i++)
		oc_riscv_write_reg(target, i, regs[i]);
}
static uint32_t oc_riscv_read_reg(struct target * target, uint32_t i)
{
	uint32_t tmpbuf;
	oc_riscv_write_Xinst(target, READREG_INST(i));
	oc_riscv_writeout(target);
	oc_riscv_readin(target, (uint8_t*)&tmpbuf, 4);
	return tmpbuf;
}
static uint64_t oc_riscv_read_2regs(struct target * target, uint32_t i, uint32_t j)
{
	uint64_t tmpbuf;
	oc_riscv_write_Xinst(target, READREG_INST(i));
	oc_riscv_write_Xinst(target, READREG_INST(j));
	oc_riscv_writeout(target);
	oc_riscv_readin(target, (uint8_t*)&tmpbuf, sizeof(tmpbuf));
	return tmpbuf;
}

static struct oc_riscv_common *target_to_oc_riscv(struct target *target)
{
	return target->arch_info;
}

static int oc_riscv_adapter_read_memory(struct target *target,
		uint32_t addr, int size, int count, void *buf)
{
	int i;
	uint32_t tmpbuf[count];
	uint32_t x1 = oc_riscv_read_reg(target, 1);
	
	oc_riscv_write_reg(target, 1, addr);
	for(i=0; i<count; i++) {
		switch(size) {
		case 1: oc_riscv_write_Xinst(target, READOPB(i)); break;
		case 2: oc_riscv_write_Xinst(target, READOPW(i*2)); break;
		case 4: oc_riscv_write_Xinst(target, READOPL(i*4)); break;
		}
	}
	oc_riscv_write_reg(target, 1, x1);
	oc_riscv_writeout(target);
	oc_riscv_readin(target, (uint8_t*)tmpbuf, count*4);
	for(i=0; i<count; i++) {
		switch(size) {
		case 1: ((uint8_t*)buf)[i] = tmpbuf[i]; break;
		case 2: ((uint16_t*)buf)[i] = tmpbuf[i]; break;
		case 4: ((uint32_t*)buf)[i] = tmpbuf[i]; break;
		}
	}
	return ERROR_OK;
}

static int oc_riscv_adapter_write_memory(struct target *target,
		uint32_t addr, int size, int count, const void *buf)
{
	int i, j;
	uint32_t out;
	uint64_t x1_2 = oc_riscv_read_2regs(target, 1, 2);
	oc_riscv_write_reg(target, 2, addr);
	for(i=0, j=0; i<count; i++, j++) {
		switch(size) {
		case 1: out = ((uint8_t*)buf)[i]; break;
		case 2: out = ((uint16_t*)buf)[i]; break;
		case 4: out = ((uint32_t*)buf)[i]; break;
		default: out = 0;
		}
		oc_riscv_write_reg(target, 1, out);
		LOG_DEBUG("write %08x=%08x", addr+i*size, out);
		switch(size) {
		case 1: oc_riscv_write_Einst(target, WRITEOPB(j)); break;
		case 2: oc_riscv_write_Einst(target, WRITEOPW(j*2)); break;
		case 4: oc_riscv_write_Einst(target, WRITEOPL(j*4)); break;
		}
		if(j>=0x40) {
			addr += j*size;
			j=0;
			oc_riscv_write_reg(target, 1, addr);
			oc_riscv_writeout(target);
		}
	}
	oc_riscv_write_reg(target, 1, x1_2 & 0xFFFFFFFF);
	oc_riscv_write_reg(target, 2, x1_2 >> 32);
	oc_riscv_writeout(target);
	return ERROR_OK;
}

static int oc_riscv_read_regs(struct target *target, uint32_t regs[])
{
	oc_riscv_gen_regs(target);
	oc_riscv_writeout(target);
	oc_riscv_readin(target, (uint8_t *)regs, OC_RISCV_NUM_REGS*4);
	return ERROR_OK;
}

static int oc_riscv_write_regs(struct target *target, uint32_t regs[])
{
	uint32_t pc = oc_riscv_read_pc(target);
	oc_riscv_gen_reg_writes(target, pc, regs);
	oc_riscv_writeout(target);
	return ERROR_OK;
}

static int oc_riscv_get_core_reg(struct reg *reg)
{
	struct oc_riscv_core_reg *oc_riscv_reg = reg->arch_info;
	struct target *target = oc_riscv_reg->target;
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);

	return oc_riscv->read_core_reg(target, oc_riscv_reg->num);
}

static int oc_riscv_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct oc_riscv_core_reg *oc_riscv_reg = reg->arch_info;
	struct target *target = oc_riscv_reg->target;
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);
	LOG_DEBUG("set core reg %s = %08x", reg->name, *(uint32_t*)buf);
        memcpy(reg->value, buf, 4);
	oc_riscv->write_core_reg(target, oc_riscv_reg->num);
	return oc_riscv_restore_context(target);
}

static int oc_riscv_save_context(struct target *target)
{
	unsigned int i;

	/* get pointers to arch-specific information */
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);

	/* read core registers */
	oc_riscv_read_regs(target, oc_riscv->core_regs);

	for (i = 0; i < OC_RISCV_NUM_REGS; i++) {
		oc_riscv->read_core_reg(target, i);
	}

	return ERROR_OK;
}

static int oc_riscv_restore_context(struct target *target)
{
	unsigned int i;

	/* get pointers to arch-specific information */
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);

	for (i = 0; i < OC_RISCV_NUM_REGS; i++) {
		if (oc_riscv->core_cache->reg_list[i].dirty)
			oc_riscv->write_core_reg(target, i);
	}

	/* write core regs */
	oc_riscv_write_regs(target, oc_riscv->core_regs);

	return ERROR_OK;
}

static int oc_riscv_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count,
		const uint8_t *buffer)
{
	LOG_DEBUG("address: 0x%8.8" TARGET_PRIxADDR
		  ", size: 0x%8.8" PRIx32
		  ", count: 0x%8.8" PRIx32,
		  address, size, count);

	int retval = oc_riscv_adapter_write_memory(target, address, size, count, buffer);
	
	if (retval != ERROR_OK)
		return ERROR_TARGET_FAILURE;

	return retval;
}

static int oc_riscv_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("address: 0x%8.8" TARGET_PRIxADDR
		  ", size: 0x%8.8" PRIx32
		  ", count: 0x%8.8" PRIx32,
		  address, size, count);

	if (target->state != TARGET_HALTED)
		LOG_WARNING("target not halted");

	int retval;
	retval = oc_riscv_adapter_read_memory(target, address, size, count, buffer);

	if (retval != ERROR_OK)
		return ERROR_TARGET_FAILURE;

	return retval;
}

static int oc_riscv_init(struct command_context *cmd_ctx, struct target *target)
{
	oc_riscv_build_reg_cache(target);

	target->state = TARGET_UNKNOWN;

	return ERROR_OK;
}

static int oc_riscv_devicename(struct target *target)
{
        char devicename[9];
	
	target->state = TARGET_UNKNOWN;
	
	oc_riscv_write_byte(target, 'I');
	oc_riscv_writeout(target);
	devicename[8]=0;
	oc_riscv_readin(target, (uint8_t*)devicename, 8);
	LOG_DEBUG("Read Devicename = %s", devicename);
	
	if(!strncmp(devicename, "1¢RISCV", 8))
		return ERROR_OK;
	else
		return ERROR_TARGET_FAILURE;
}

static int oc_riscv_state(struct target *target, uint8_t query)
{
	uint8_t state;

	target->state = TARGET_UNKNOWN;
	
	oc_riscv_write_byte(target, query);
	oc_riscv_writeout(target);
	oc_riscv_readin(target, &state, 1);

	switch(state) {
	case 'b':
		LOG_DEBUG("oc-riscv breakpoint reached");
		target->state = TARGET_HALTED;
		target->debug_reason = DBG_REASON_BREAKPOINT;
		break;
	case 'r':
		LOG_DEBUG("oc-riscv running");
		target->state = TARGET_RUNNING;
		break;
	case 'h':
		LOG_DEBUG("oc-riscv halted");
		target->state = TARGET_HALTED;
		target->debug_reason = DBG_REASON_DBGRQ;
		break;
	default: return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int oc_riscv_halted(struct target *target, enum target_state oldstate, int retval)
{
	switch(target->state) {
	case TARGET_HALTED:
	case TARGET_DEBUG_RUNNING:
		if(oldstate != target->state) {
			retval = oc_riscv_save_context(target);
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
		break;
	default: break;
	}
	return retval;
}

static int oc_riscv_poll(struct target *target)
{
        enum target_state oldstate = target->state;
	return oc_riscv_halted(target, oldstate, oc_riscv_state(target, '?'));
}

static int oc_riscv_halt(struct target *target)
{
        enum target_state oldstate = target->state;
	return oc_riscv_halted(target, oldstate, oc_riscv_state(target, 'H'));
}

static int oc_riscv_resume(struct target *target, bool current,
		target_addr_t address, bool handle_breakpoints,
		bool debug_execution)
{
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);

	LOG_DEBUG("%d " TARGET_ADDR_FMT " %d %d", current, address,
			handle_breakpoints, debug_execution);

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if(!current) {
		memcpy(oc_riscv->core_cache->reg_list[OC_RISCV_PC].value, &address, 4);
		oc_riscv->write_core_reg(target, OC_RISCV_PC);
	}
	
	oc_riscv_restore_context(target);
	oc_riscv_write_Einst(target, JMPNOP);
	oc_riscv_writeout(target);

	int result = oc_riscv_state(target, 'R');
	return result;
}

static int oc_riscv_init_arch_info(struct target *target,
		struct oc_riscv_common *oc_riscv, struct jtag_tap *tap)
{
	target->endianness = TARGET_LITTLE_ENDIAN;
	target->arch_info = oc_riscv;
	oc_riscv->common_magic = OC_RISCV_COMMON_MAGIC;

	oc_riscv->read_core_reg = oc_riscv_read_core_reg;
	oc_riscv->write_core_reg = oc_riscv_write_core_reg;

	return ERROR_OK;
}

static int oc_riscv_target_create(struct target *target)
{

	struct oc_riscv_common *oc_riscv = calloc(1, sizeof(struct oc_riscv_common));

	oc_riscv->uart_id = 1; // default is /dev/ttyUSB1
	oc_riscv_init_arch_info(target, oc_riscv, target->tap);

	return ERROR_OK;
}

static int oc_riscv_read_core_reg(struct target *target, unsigned int num)
{
	uint32_t reg_value;

	/* get pointers to arch-specific information */
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);

	if (num >= OC_RISCV_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = oc_riscv->core_regs[num];
	memcpy(oc_riscv->core_cache->reg_list[num].value, &reg_value, 4);
	oc_riscv->core_cache->reg_list[num].valid = true;
	oc_riscv->core_cache->reg_list[num].dirty = false;

	return ERROR_OK;
}

static int oc_riscv_write_core_reg(struct target *target, unsigned int num)
{
	uint32_t reg_value;

	/* get pointers to arch-specific information */
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);

	if (num >= OC_RISCV_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	memcpy(&reg_value, oc_riscv->core_cache->reg_list[num].value, 4);
	oc_riscv->core_regs[num] = reg_value;
	oc_riscv->core_cache->reg_list[num].valid = true;
	oc_riscv->core_cache->reg_list[num].dirty = false;

	return ERROR_OK;
}

static const char *oc_riscv_get_gdb_arch(const struct target *target)
{
	return "riscv:rv32";
}

static int oc_riscv_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	LOG_DEBUG("create breakpoint type %d at %08x",
		  breakpoint->type, (uint32_t)(breakpoint->address));
	if (breakpoint->type == BKPT_SOFT) {
		uint32_t ebreak = 0x00100073;
		oc_riscv_read_memory(target, breakpoint->address,
				     4, 1, (uint8_t*)breakpoint->orig_instr);
		oc_riscv_write_memory(target, breakpoint->address,
				      4, 1, (uint8_t*)&ebreak);
		LOG_DEBUG("Set breakpoint to %08x, orig inst %08x -> %08x",
			  (uint32_t)(breakpoint->address), *(uint32_t*)breakpoint->orig_instr,
			  ebreak);
		breakpoint->is_set = true;
		return ERROR_OK;
	} else {
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
}

static int oc_riscv_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	if (breakpoint->type == BKPT_SOFT) {
		oc_riscv_write_memory(target, breakpoint->address,
				      breakpoint->length, 1, (uint8_t*)breakpoint->orig_instr);
		breakpoint->is_set = false;
		LOG_DEBUG("Remove breakpoint to %08x, orig inst %08x",
			  (uint32_t)(breakpoint->address), *(uint32_t*)breakpoint->orig_instr);
		return ERROR_OK;
	} else {
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
}

static int oc_riscv_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class)
{
	/* get pointers to arch-specific information */
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);
	unsigned int i;

	*reg_list_size = OC_RISCV_NUM_REGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	for (i = 0; i < OC_RISCV_NUM_REGS; i++)
		(*reg_list)[i] = &oc_riscv->core_cache->reg_list[i];

	return ERROR_OK;
}

static const struct reg_arch_type oc_riscv_reg_type = {
	.get = oc_riscv_get_core_reg,
	.set = oc_riscv_set_core_reg,
};

static struct reg_cache *oc_riscv_build_reg_cache(struct target *target)
{
	/* get pointers to arch-specific information */
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);

	int num_regs = OC_RISCV_NUM_REGS;
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct oc_riscv_core_reg *arch_info = malloc(
			sizeof(struct oc_riscv_core_reg) * num_regs);
	int i;

	/* Build the process context cache */
	cache->name = "riscv registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;
	oc_riscv->core_cache = cache;

	for (i = 0; i < num_regs; i++) {
		arch_info[i].num = oc_riscv_regs[i].id;
		arch_info[i].target = target;

		reg_list[i].name = oc_riscv_regs[i].name;
		reg_list[i].size = regno_size;

		reg_list[i].value = calloc(1, 4);
		reg_list[i].valid = false;
		reg_list[i].type = &oc_riscv_reg_type;
		reg_list[i].arch_info = &arch_info[i];

		reg_list[i].reg_data_type = calloc(1, sizeof(struct reg_data_type));
		if (reg_list[i].reg_data_type)
			reg_list[i].reg_data_type->type = regno_type;
		else {
			LOG_ERROR("unable to allocate reg type list");
			return NULL;
		}

		reg_list[i].dirty = false;
		reg_list[i].group = regno_group;
		reg_list[i].number = oc_riscv_regs[i].id;
		reg_list[i].exist = true;
		reg_list[i].caller_save = true;	/* gdb defaults to true */

		reg_list[i].feature = &feature_cpu;
	}

	return cache;
}

static void oc_riscv_free_reg_cache(struct target *target)
{
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);
	struct reg_cache *cache;
	struct reg *reg;
	unsigned int i;

	cache = oc_riscv->core_cache;

	if (!cache)
		return;

	for (i = 0; i < cache->num_regs; i++) {
		reg = &cache->reg_list[i];

		free(reg->feature);
		free(reg->reg_data_type);
		free(reg->value);
	}

	free(cache->reg_list[0].arch_info);
	free(cache->reg_list);
	free(cache);

	oc_riscv->core_cache = NULL;
}

static void oc_riscv_deinit(struct target *target)
{
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);

	oc_riscv_free_reg_cache(target);

	free(oc_riscv);
}

static int oc_riscv_arch_state(struct target *target)
{
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);

	LOG_USER("target halted due to %s, pc: 0x%8.8" PRIx32,
		 debug_reason_name(target),
		 *(uint32_t*)(oc_riscv->core_cache->reg_list[OC_RISCV_PC].value));

	return ERROR_OK;
}

static int oc_riscv_step(struct target *target, bool current,
		target_addr_t address, bool handle_breakpoints)
{
	LOG_DEBUG("%d " TARGET_ADDR_FMT " %d", current, address,
			handle_breakpoints);

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	oc_riscv_restore_context(target);
	oc_riscv_write_Einst(target, JMPNOP);
	oc_riscv_writeout(target);

	int result = oc_riscv_state(target, 'S');
	return result;
}

static int oc_riscv_examine(struct target *target)
{
	/* get pointers to arch-specific information */
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);

	LOG_DEBUG("Examine OC RISCV connection");

	if (!oc_riscv->uart_configured) {
		LOG_DEBUG("Open Target");
		oc_riscv_open_target(target);
	}
	int retval = oc_riscv_devicename(target);
	if(retval == ERROR_OK)
		target_set_examined(target);

	return retval;
}

static int oc_riscv_jim_configure(struct target *target, struct jim_getopt_info *goi)
{
	struct oc_riscv_common *oc_riscv = target_to_oc_riscv(target);
	jim_wide w;
	int e;
	const char *arg;

	arg = Jim_GetString(goi->argv[0], NULL);
	if (!strcmp(arg, "-uart")) {
		e = jim_getopt_string(goi, &arg, NULL);
		if (e != JIM_OK)
			return e;

		if (goi->argc == 0) {
			Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv,
					"-uart ?device-id? ...");
			return JIM_ERR;
		}

		e = jim_getopt_wide(goi, &w);
		if (e != JIM_OK)
			return e;

		oc_riscv->uart_id = (int)w;
		LOG_DEBUG("uart=/dev/ttyUSB%d" PRIx32, oc_riscv->uart_id);
		return JIM_OK;
	}
	return JIM_CONTINUE;
}

static const struct command_registration oc_riscv_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration oc_riscv_command_handlers[] = {
	{
		.name = "oc_riscv",
		.mode = COMMAND_ANY,
		.help = "oc_riscv command group",
		.usage = "",
		.chain = oc_riscv_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type oc_riscv_target = {
	.name = "oc_riscv",

	.poll = oc_riscv_poll,
	.arch_state = oc_riscv_arch_state,

	.halt = oc_riscv_halt,
	.resume = oc_riscv_resume,
	.step = oc_riscv_step,
	.add_breakpoint = oc_riscv_add_breakpoint,
	.remove_breakpoint = oc_riscv_remove_breakpoint,

	.get_gdb_arch = oc_riscv_get_gdb_arch,
	.get_gdb_reg_list = oc_riscv_get_gdb_reg_list,

	.read_memory = oc_riscv_read_memory,
	.write_memory = oc_riscv_write_memory,

	.commands = oc_riscv_command_handlers,
	.target_create = oc_riscv_target_create,
	.init_target = oc_riscv_init,
	.examine = oc_riscv_examine,

	.deinit_target = oc_riscv_deinit,
	.target_jim_configure = oc_riscv_jim_configure,
};

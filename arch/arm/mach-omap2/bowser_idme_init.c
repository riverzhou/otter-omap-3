/*
 * board IDME driver
 *
 * Copyright (C) 2011 Amazon Inc., All Rights Reserved.
 *
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sysdev.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <linux/proc_fs.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <mach/bowser_idme_init.h>

#define DRIVER_VER "1.1"

#define DRIVER_INFO "Lab126 Idme driver version " DRIVER_VER

#define IDME_SERIAL_PROCNAME	"serial"
#define IDME_PROCNAME_BOARDID	"board_id"
#define IDME_PROCNAME_PCBSN	"pcbsn"
#define IDME_PROCNAME_MACADDR	"mac_addr"
#define IDME_PROCNAME_BTMACADDR	"bt_mac_addr"
#define IDME_PROCNAME_MACSEC	"mac_sec"
#define IDME_PROCNAME_BOOTMODE	"bootmode"
#define IDME_PROCNAME_POSTMODE	"postmode"
#define IDME_PROCNAME_BOOTCOUNT "boot_count"
#define IDME_PROCNAME_PROD_NAME	"product_name"
#define IDME_PROCNAME_PROD_NAME_EXTRA "product_name_extra"

#ifndef MIN
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif

int idme_get_board_revision(void);
char *idme_get_board_type_string(void);
	
#define PROD_NAME_EXTRA_DUMMY 0
#define PROD_NAME_EXTRA_ENG 1
#define PROD_NAME_EXTRA_PROD 2
	
const char *prod_name_extra_string[] = {
	"Dummy\0", 
	"Eng\0", 
	"Prod\0"
};

#define MAX_PROD_NAME_STRING_LEN 256
#if 0
static char prod_name_string[MAX_PROD_NAME_STRING_LEN];
#endif
static int prod_name_extra_index = PROD_NAME_EXTRA_DUMMY;

#define BOARD_TYPE_OFFSET 0   //the top 3 bytes are the board type
#define BOARD_TYPE_LEN 3
#define BOARD_REV_OFFSET (BOARD_TYPE_OFFSET + BOARD_TYPE_LEN)
#define BOARD_REV_LEN 2
#define BOARD_VARIANT_OFFSET (BOARD_REV_OFFSET + BOARD_REV_LEN)
#define BOARD_VARIANT_LEN 2

#define BOWSER_BOARD_TYPE "000"
#define JEM_BOARD_TYPE "001"
#define TATE_BOARD_TYPE "002"
#define RADLEY_BOARD_TYPE "804"

int idme_get_board_revision(void);
char *idme_get_board_type_string(void);

static int proc_id_read(char *page, char **start, off_t off, int count,
				int *eof, void *data, char *id)
{
	strcpy(page, id);
	*eof = 1;

	return strlen(page);
}

static int proc_bootcount_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
        unsigned long bootcount = 0;
        memcpy((char *)&bootcount, system_postmode, sizeof(unsigned long));
        sprintf(page, "%lu\n", bootcount);
	*eof = 1;
	return strlen(page);
}
#define PROC_ID_READ(id) proc_id_read(page, start, off, count, eof, data, id)

static int proc_serial_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
    return PROC_ID_READ(system_serial16);
}

static int proc_board_id_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
    return PROC_ID_READ(system_rev16);
}

static int proc_mac_address_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
    return PROC_ID_READ(system_mac_addr);
}

static int proc_bt_mac_address_read(char *page, char **start, off_t off, int count,
                                int *eof, void *data)
{
    return PROC_ID_READ(system_bt_mac_addr);
}

static int proc_mac_secret_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
    return PROC_ID_READ(system_mac_sec);
}

static int proc_bootmode_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
    return PROC_ID_READ(system_bootmode);
}

static int proc_postmode_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
    return PROC_ID_READ(system_postmode);
}

static int proc_prod_name_func(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
        char *str = idme_get_board_type_string();
        if ((str) && (strlen(str) < PAGE_SIZE)){
                memcpy(page, str, strlen(str));
                *eof = 1;
                return strlen(str);
        }
        return 0;
}

static int proc_prod_name_extra_func(char *page, char **start, off_t off, int count,
			int *eof, void *data)
{
	const char *str = prod_name_extra_string[prod_name_extra_index];
	if ((str) && (strlen(str) < PAGE_SIZE)){
		memcpy(page, str, strlen(str));
		*eof = 1;
		return strlen(str);
	}
	return 0;
}

static int __init bowser_idme_init_proc(void)
{
	struct proc_dir_entry *proc_serial = create_proc_entry(IDME_SERIAL_PROCNAME, S_IRUGO, NULL);
	struct proc_dir_entry *proc_board_id = create_proc_entry(IDME_PROCNAME_BOARDID, S_IRUGO, NULL);
	struct proc_dir_entry *proc_mac_address = create_proc_entry(IDME_PROCNAME_MACADDR, S_IRUGO, NULL);
	struct proc_dir_entry *proc_bt_mac_address = create_proc_entry(IDME_PROCNAME_BTMACADDR, S_IRUGO, NULL);
	struct proc_dir_entry *proc_mac_secret = create_proc_entry(IDME_PROCNAME_MACSEC, S_IRUSR, NULL);
	struct proc_dir_entry *proc_bootmode = create_proc_entry(IDME_PROCNAME_BOOTMODE, S_IRUGO, NULL);
	struct proc_dir_entry *proc_postmode = create_proc_entry(IDME_PROCNAME_POSTMODE, S_IRUGO, NULL);
	struct proc_dir_entry *proc_bootcount = create_proc_entry(IDME_PROCNAME_BOOTCOUNT, S_IRUGO, NULL);
	struct proc_dir_entry *proc_prod_name = create_proc_entry(IDME_PROCNAME_PROD_NAME, S_IRUGO, NULL);
	struct proc_dir_entry *proc_prod_name_extra = create_proc_entry(IDME_PROCNAME_PROD_NAME_EXTRA, S_IRUGO, NULL);

	if (proc_serial != NULL) {
		proc_serial->data = NULL;
		proc_serial->read_proc = proc_serial_read;
		proc_serial->write_proc = NULL;
	}

	if (proc_board_id != NULL) {
		proc_board_id->data = NULL;
		proc_board_id->read_proc = proc_board_id_read;
		proc_board_id->write_proc = NULL;
	}

	if (proc_mac_address != NULL) {
		proc_mac_address->data = NULL;
		proc_mac_address->read_proc = proc_mac_address_read;
		proc_mac_address->write_proc = NULL;
	}


	if (proc_mac_secret != NULL) {
		proc_mac_secret->data = NULL;
		proc_mac_secret->read_proc = proc_mac_secret_read;
		proc_mac_secret->write_proc = NULL;
	}

	if (proc_bootmode != NULL) {
		proc_bootmode->data = NULL;
		proc_bootmode->read_proc = proc_bootmode_read;
		proc_bootmode->write_proc = NULL;
	}

	if (proc_postmode != NULL) {
		proc_postmode->data = NULL;
		proc_postmode->read_proc = proc_postmode_read;
		proc_postmode->write_proc = NULL;
	}

	if (proc_bootcount != NULL) {
		proc_bootcount->data = NULL;
		proc_bootcount->read_proc = proc_bootcount_read;
		proc_bootcount->write_proc = NULL;
	}
	if (proc_bt_mac_address != NULL) {
		proc_bt_mac_address->data = NULL;
		proc_bt_mac_address->read_proc = proc_bt_mac_address_read;
		proc_bt_mac_address->write_proc = NULL;
	}


	if (proc_prod_name != NULL) {
		proc_prod_name->data = NULL;
		proc_prod_name->read_proc = proc_prod_name_func;
		proc_prod_name->write_proc = NULL;
	}

        if (proc_prod_name_extra != NULL) {
                proc_prod_name_extra->data = NULL;
                proc_prod_name_extra->read_proc = proc_prod_name_extra_func;
                proc_prod_name_extra->write_proc = NULL;
        }
	return 0;
}

module_init(bowser_idme_init_proc);




static int inline is_bowser_board_type(const unsigned char *pbid)
{
    if (strncmp(BOWSER_BOARD_TYPE, pbid, BOARD_TYPE_LEN) == 0)
        return 1;
    return 0;
}

/* 
   my own strtol conversion instead of using kernel interface which
   is more overhead

   Assumption: the board revision is always 2 digits

   return -1 if error
*/
#if 0
static int rev_str_to_num(const unsigned char *prev)
{
    int rev = 0;
    if (!prev){
        pr_err("Error, rev_str_to_num get invalid pointer\n");
        return -1;
    }

    rev = prev[1] - '0';
    rev += (prev[0] - '0') * 10;
    return rev;
}
#endif

int mystrtoi(const char *s, const char *e)
{
    int result = 0, value = 0;
    /* first remove all leading 0s */
    while(*s == '0' && s < e){
        s++;
    }

    while(s < e){
        value = *s - '0';
        result = result * 10 + value;
        s++;
    }
    return result;
}

/* return the variant of the board */
int idme_get_board_type(void)
{
    int board_type = 0;
    board_type = mystrtoi(&system_rev16[BOARD_TYPE_OFFSET],
                          &system_rev16[BOARD_TYPE_OFFSET+BOARD_TYPE_LEN]);
    return board_type;
}

EXPORT_SYMBOL(idme_get_board_type);


char *idme_get_board_type_string(void)
{
        switch(idme_get_board_type()){
        case BOARD_TYPE_OTTER2_DUMMY:
		prod_name_extra_index = PROD_NAME_EXTRA_DUMMY;
                return "Otter2 Dummy\0";
        case BOARD_TYPE_OTTER2_ENG:
		prod_name_extra_index = PROD_NAME_EXTRA_ENG;
                return "Otter2 Eng\0";
        case BOARD_TYPE_OTTER2_PROD:
		prod_name_extra_index = PROD_NAME_EXTRA_PROD;
                return "Otter2 Prod\0";
        }
        return "Otter2 Dummy\0";
}  



static void print_bootcount(void)
{
        unsigned long count = 0;
        memcpy((char *)&count, system_postmode, sizeof(unsigned long));
        printk("boot count = %lu\n", count);
}


static void self_test(void)
{
	/* TTX-1812: Privacy- Customer Name and DSN is logged in the logs
	   Remove logging DSN */
	/* printk("Bowser serial number - %s\n", system_serial16); */
        printk("Bowser board id - %s\n", system_rev16);

        /* self test code just to dump the idme board info */
        printk("\nboard type = %d\n", idme_get_board_type());
        print_bootcount();
}


void bowser_init_idme(void)
{
    printk(DRIVER_INFO "\n");

    self_test();
}

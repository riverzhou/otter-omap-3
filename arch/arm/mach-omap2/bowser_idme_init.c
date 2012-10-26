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
#define IDME_PROCNAME_GYROCAL	"gyrocal"
#define IDME_PROCNAME_PROD_NAME	"product_name"

#define PRODUCT_FEATURE_NAME_GPS "gps"
#define PRODUCT_FEATURE_NAME_WAN "wan"

#define PRODUCT_FEATURE_STRING_GPS " "
#define PRODUCT_FEATURE_STRING_WAN " "

#ifndef MIN
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif


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

int __initdata board_has_gps(void);
int __initdata board_has_wan(void);

static int proc_id_read(char *page, char **start, off_t off, int count,
				int *eof, void *data, char *id)
{
	strcpy(page, id);
	*eof = 1;

	return strlen(page);
}

static int proc_bootcount_read(char *page, char **start, off_t off, int count,
				int *eof, void *data, char *id)
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

static int proc_gyrocal_read(char *page, char **start, off_t off, int count,
				int *eof, void *data, char *id)
{
	unsigned int len = 36;
	/* Copy raw data, instead of a string.*/
	memcpy(page, system_gyro_cal, len);
	*eof = 1;

	return len;
}

static int proc_prod_name_func(char *page, char **start, off_t off, int count,
				int *eof, void *data, char *id)
{
        char *str = idme_get_board_type_string();
        if ((str) && (strlen(str) < PAGE_SIZE)){
                memcpy(page, str, strlen(str));
                *eof = 1;
                return strlen(str);
        }
        return 0;
}


static int proc_pfeature_gps_name_func(char *page, char **start, off_t off, int count,
				int *eof, void *data, char *id)
{
        char str[] = PRODUCT_FEATURE_STRING_GPS;
        if ((str) && (strlen(str) < PAGE_SIZE)){
                memcpy(page, str, strlen(str));
                *eof = 1;
                return strlen(str);
        }
        return 0;
}
static int proc_pfeature_wan_name_func(char *page, char **start, off_t off, int count,
				int *eof, void *data, char *id)
{
        char str[] = PRODUCT_FEATURE_STRING_WAN;
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
	struct proc_dir_entry *proc_gyrocal = create_proc_entry(IDME_PROCNAME_GYROCAL, S_IRUGO, NULL);
	struct proc_dir_entry *proc_prod_name = create_proc_entry(IDME_PROCNAME_PROD_NAME, S_IRUGO, NULL);
        static struct proc_dir_entry *proc_product_features_dir;



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

	if (proc_gyrocal != NULL) {
		proc_gyrocal->data = NULL;
		proc_gyrocal->read_proc = proc_gyrocal_read;
		proc_gyrocal->write_proc = NULL;
	}

	if (proc_prod_name != NULL) {
		proc_prod_name->data = NULL;
		proc_prod_name->read_proc = proc_prod_name_func;
		proc_prod_name->write_proc = NULL;
	}


        proc_product_features_dir = proc_mkdir("product_features", NULL);
        if (proc_product_features_dir)
	{
		if (board_has_gps()) {
		struct proc_dir_entry *proc_gps_name = create_proc_entry(PRODUCT_FEATURE_NAME_GPS, S_IRUGO, proc_product_features_dir);
		if (proc_gps_name != NULL) {
				proc_gps_name->data = NULL;
				proc_gps_name->read_proc = proc_pfeature_gps_name_func;
				proc_gps_name->write_proc = NULL;
			}
		}
		if (board_has_wan()) {
			struct proc_dir_entry *proc_wan_name = create_proc_entry(PRODUCT_FEATURE_NAME_WAN, S_IRUGO, proc_product_features_dir);
			if (proc_wan_name != NULL) {
				proc_wan_name->data = NULL;
				proc_wan_name->read_proc = proc_pfeature_wan_name_func;
				proc_wan_name->write_proc = NULL;
			}
		}

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

int is_radley_device(void)
{
        if (strncmp(RADLEY_BOARD_TYPE, &system_rev16[BOARD_TYPE_OFFSET], BOARD_TYPE_LEN) == 0 )
                return 1;
        return 0;
}

int __initdata board_has_usb_host(void)
{
        if (is_radley_device()) return 1;
        return 0;
}



/* 
   my own strtol conversion instead of using kernel interface which
   is more overhead

   Assumption: the board revision is always 2 digits

   return -1 if error
*/
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
        

/* return the variant of the board */
int idme_get_board_variant(void)
{

        return mystrtoi(&system_rev16[BOARD_VARIANT_OFFSET],
                        &system_rev16[BOARD_VARIANT_OFFSET+BOARD_VARIANT_LEN]);
}

EXPORT_SYMBOL(idme_get_board_variant);


int idme_get_board_revision(void)
{
        return mystrtoi(&system_rev16[BOARD_REV_OFFSET],
                        &system_rev16[BOARD_REV_OFFSET+BOARD_REV_LEN]);
}


EXPORT_SYMBOL(idme_get_board_revision);

char *idme_get_board_type_string(void)
{
        switch(idme_get_board_type()){
        case BOARD_TYPE_BOWSER:
                switch(idme_get_board_revision()){
                case 1: return "Bowser 1\0";
                case 2: return "Bowser 2\0";
                case 3: return "Bowser 3\0";                
                case 4: return "Bowser 4\0";
                case 5: return "Bowser 5\0";
                case 6: return "Bowser 6\0";                
                case 7: return "Bowser 7\0";
                default: return "Unknown\0";
                }
                break;
        case BOARD_TYPE_JEM:
                switch(idme_get_board_revision()){
                case 1: return "Jem EVT 1\0";
                case 2: return "Jem EVT 1.2 Wifi\0";
                case 3: return "Jem EVT 1.2 WAN\0";
                default: return "Unknown\0";
                }
                break;
        case BOARD_TYPE_JEM_WIFI:
                switch(idme_get_board_revision()){
                case 1: return "Jem EVT2 Wifi\0";
                case 2: return "Jem EVT3 Wifi\0";
                default: return "Unknown\0";
                }
                break;
        case BOARD_TYPE_JEM_WAN:
                switch(idme_get_board_revision()){
                case 1: return "Jem EVT2 WAN\0";
                case 2: return "Jem EVT3 WAN\0";
                default: return "Unknown\0";
                }
                break;
        case BOARD_TYPE_TATE:
                return "Tate PreEVT2.1\0";
        case BOARD_TYPE_TATE_EVT2_1:
                switch(idme_get_board_revision()){
                case 4: return "Tate EVT2.1\0";
                case 5: return "Tate EVT3\0";
                case 6: return "Tate EVT3HS2\0";
                case 7: return "Tate DVT\0";
                case 8: return "Tate PVT\0";
		default: return "Unknown\0";
                }
        case BOARD_TYPE_RADLEY:
                switch(idme_get_board_revision()){
		case 0: return "Radley EVT0\0";
                case 1: return "Radley EVT1\0";
                case 2: return "Radley EVT2\0";
                default: return "Unknown\0";
                }
        }
        return NULL;
}  


int __initdata board_has_gps(void)
{
        switch(idme_get_board_type()){
        	case BOARD_TYPE_BOWSER:
	        case BOARD_TYPE_JEM_WAN:
        	case BOARD_TYPE_RADLEY:
		return 1;
	}
        return 0;
}

int __initdata board_has_wan(void)
{
        switch(idme_get_board_type()){
        	case BOARD_TYPE_BOWSER:
	        case BOARD_TYPE_JEM_WAN:
        	case BOARD_TYPE_RADLEY:
		return 1;
	}
        return 0;
}



int idme_query_board_type(const enum idme_board_type bt)
{
	int result = 0;

	switch(bt){
		/*Bowser board*/
		case IDME_BOARD_TYPE_BOWER1:
			if ((idme_get_board_type() == BOARD_TYPE_BOWSER) &&
					(idme_get_board_revision() == 1)){
				printk("Bowser Rev.1\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_BOWER2:
			if ((idme_get_board_type() == BOARD_TYPE_BOWSER) &&
					(idme_get_board_revision() == 2)){
				printk("Bowser Rev.2\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_BOWER3:
			if ((idme_get_board_type() == BOARD_TYPE_BOWSER) &&
					(idme_get_board_revision() == 3)){
				printk("Bowser Rev.3\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_BOWER4:
			if ((idme_get_board_type() == BOARD_TYPE_BOWSER) &&
					(idme_get_board_revision() == 4)){
				printk("Bowser Rev.4\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_BOWER5:
			if ((idme_get_board_type() == BOARD_TYPE_BOWSER) &&
					(idme_get_board_revision() == 5)){
				printk("Bowser Rev.5\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_BOWER6:
			if ((idme_get_board_type() == BOARD_TYPE_BOWSER) &&
					(idme_get_board_revision() == 6)){
				printk("Bowser Rev.6\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_BOWER7:
			if ((idme_get_board_type() == BOARD_TYPE_BOWSER) &&
					(idme_get_board_revision() == 7)){
				printk("Bowser Rev.7\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_BOWER8:
			if ((idme_get_board_type() == BOARD_TYPE_BOWSER) &&
					(idme_get_board_revision() == 8)){
				printk("Bowser Rev.8\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_BOWER9:
			if ((idme_get_board_type() == BOARD_TYPE_BOWSER) &&
					(idme_get_board_revision() == 9)){
				printk("Bowser Rev.9\n");
				result = 1;
			}
			break;
		/*Jem device*/
		case IDME_BOARD_TYPE_JEM_PROTO:
			if ((idme_get_board_type() == BOARD_TYPE_JEM) &&
					(idme_get_board_revision() == 0)){
				printk("Jem Proto\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_JEM_EVT1:
			if ((idme_get_board_type() == BOARD_TYPE_JEM) &&
					(idme_get_board_revision() == 1)){
				printk("Jem EVT 1.0\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_JEM_EVT1_2_WIFI:
			if ((idme_get_board_type() == BOARD_TYPE_JEM) &&
					(idme_get_board_revision() == 2)){
				printk("Jem EVT 1.2 WIFI\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_JEM_EVT1_2_WAN:
			if ((idme_get_board_type() == BOARD_TYPE_JEM) &&
					(idme_get_board_revision() == 3)){
				printk("Jem EVT 1.2 WAN\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_JEM_EVT2_WIFI:
			if ((idme_get_board_type() == BOARD_TYPE_JEM_WIFI) &&
					(idme_get_board_revision() == 1)){
				printk("Jem EVT 2.0 WIFI\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_JEM_EVT2_WAN:
			if ((idme_get_board_type() == BOARD_TYPE_JEM_WAN) &&
					(idme_get_board_revision() == 1)){
				printk("Jem EVT 2.0 WAN\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_JEM_EVT3_WIFI:
			if ((idme_get_board_type() == BOARD_TYPE_JEM_WIFI) &&
					(idme_get_board_revision() == 2)){
				printk("Jem EVT 3.0 WIFI\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_JEM_EVT3_WAN:
			if ((idme_get_board_type() == BOARD_TYPE_JEM_WAN) &&
					(idme_get_board_revision() == 2)){
				printk("Jem EVT 3.0 WAN\n");
				result = 1;
			}
			break;
		/*Tate device*/
		case IDME_BOARD_TYPE_TATE_PROTO:
			if ((idme_get_board_type() == BOARD_TYPE_TATE) &&
					(idme_get_board_revision() == 0)){
				printk("Tate EVT Proto\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_TATE_EVT_PRE:
			if ((idme_get_board_type() == BOARD_TYPE_TATE) &&
					(idme_get_board_revision() == 1)){
				printk("Tate EVT Pre\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_TATE_EVT1:
		case IDME_BOARD_TYPE_TATE_EVT1_0A:
		case IDME_BOARD_TYPE_TATE_EVT1_1:
			if ((idme_get_board_type() == BOARD_TYPE_TATE) &&
					(idme_get_board_revision() == 2)){
				printk("Tate EVT 1.0/1.0a/1.1\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_TATE_EVT2:
			if ((idme_get_board_type() == BOARD_TYPE_TATE) &&
					(idme_get_board_revision() == 3)){
				printk("Tate EVT 2.0\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_TATE_EVT2_1:
			if ((idme_get_board_type() == BOARD_TYPE_TATE_EVT2_1) &&
					(idme_get_board_revision() == 1)){
				printk("Tate EVT 2.1\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_TATE_EVT3:
			if ((idme_get_board_type() == BOARD_TYPE_TATE_EVT3) &&
					(idme_get_board_revision() == 2)){
				printk("Tate EVT 3.0\n");
				result = 1;
			}
			break;
		/*Radley device*/
		case IDME_BOARD_TYPE_RADLEY_EVT1:
			if ((idme_get_board_type() == BOARD_TYPE_RADLEY) &&
					(idme_get_board_revision() == 1)){
				printk("Radley EVT 1.0\n");
				result = 1;
			}
			break;
		case IDME_BOARD_TYPE_RADLEY_EVT2:
			if ((idme_get_board_type() == BOARD_TYPE_RADLEY) &&
					(idme_get_board_revision() == 2)){
				printk("Radley EVT 2.0\n");
				result = 1;
			}
			break;
		default:
			printk("Unknown Board Type [%d]\n",bt);
			break;
	}
	return result;
}

EXPORT_SYMBOL(idme_query_board_type);

// return true if the board is bowser 4
int is_bowser_rev4(void)
{
        /* checking if it is bowser board */
	if (is_bowser_board_type(&system_rev16[BOARD_TYPE_OFFSET])){
                int rev = rev_str_to_num(&system_rev16[BOARD_REV_OFFSET]);
                printk("Eric: bowser rev %d\n", rev);
                if (rev >= 4)
                        return 1;
        }
        return 0;
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
        printk("\nboard revision = %d\n", idme_get_board_revision());
        printk("\nboard variant = %d\n", idme_get_board_variant());

        print_bootcount();
}


void bowser_init_idme(void)
{
        printk(DRIVER_INFO "\n");
        
        self_test();
}

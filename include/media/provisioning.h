/*
 * provisioning.h
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _PROVISIONING_H_
#define _PROVISIONING_H_


/********************************/
/* Structures related to ioctl  */
/********************************/

/* PEK key size in 32-bit words: 32 bytes */
#define PEK_KEY_SIZE	8

#include <linux/ioctl.h>
#include <linux/types.h>

struct pek_encrypt_control {  
	uint32_t in_key[PEK_KEY_SIZE];     /* 1. pek key input:  PEK(CEK)=AES_encrypt(PEK, CEK) */ 
	uint32_t *out_key;   /* 2. pek key output: PEK(KEK)=AES_encrypt(PEK, KEK) */
	uint32_t magic_key;
};

/* Provisioning ioctl */
#define PROVISIONING_IOCTL_MAGIC 'p'
#define PROVISIONING_PEK_ENCRYPT_KEY 	_IOWR(PROVISIONING_IOCTL_MAGIC, 6, \
					struct pek_encrypt_control)

#endif /* _PROVISIONING_H_ */

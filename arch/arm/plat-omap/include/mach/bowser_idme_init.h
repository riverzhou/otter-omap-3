/*
 * board IDME driver header file
 *
 * Copyright (C) 2011 Amazon Inc., All Rights Reserved.
 *
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _BOWSER_IDME_INIT_H_
#define _BOWSER_IDME_INIT_H_

#define BOARD_TYPE_OTTER2_DUMMY	805
#define BOARD_TYPE_OTTER2_ENG	806
#define BOARD_TYPE_OTTER2_PROD	807

enum idme_board_type{
	IDME_BOARD_TYPE_OTTER2_DUMMY=805,
	IDME_BOARD_TYPE_OTTER2_ENG,
	IDME_BOARD_TYPE_OTTER2_PROD
};


void bowser_init_idme(void);
int is_bowser_rev4(void);
#endif

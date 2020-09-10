#####################################################################
# Copyright (c) 2001 Point Grey Research, Inc. All Rights Reserved.
#
# This software is the confidential and proprietary information of Point
# Grey Research, Inc. ("Confidential Information").  You shall not
# disclose such Confidential Information and shall use it only in
# accordance with the terms of the license agreement you entered into
# with PGR.
#
# PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
# SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
# SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
# THIS SOFTWARE OR ITS DERIVATIVES.
#####################################################################
#
# $Id: Makefile,v 1.1 2001-09-20 23:26:00 gchow Exp $
# $Author: gchow $
# $Revision: 1.1 $
# $Date: 2001-09-20 23:26:00 $
#
#####################################################################


PKG =	common 

all: bin

bin clean installbin:
	for pkg in $(PKG) ; do \
	    ( echo $$pkg:$@ ; cd $$pkg ; $(MAKE) $(MFLAGS) $@ ) ;\
	done

	

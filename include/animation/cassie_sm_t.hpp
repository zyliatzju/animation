/*
 * Copyright (c) 2019 Agility Robotics
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef CASSIE_SM_T_H
#define CASSIE_SM_T_H

#define CASSIE_SM_IN_T_PACKED_LEN 4 
#define CASSIE_SM_OUT_T_PACKED_LEN 6 

#include <stdbool.h>


typedef struct {
  short track_ani; //2
  short in_ani; //2
  short ani_finished; //2
} cassie_sm_out_t;

typedef struct {
  int motion_id; //4
} cassie_sm_in_t;


#ifdef __cplusplus
extern "C" {
#endif

void pack_cassie_sm_in_t(const cassie_sm_in_t *bus, unsigned char *bytes);
void unpack_cassie_sm_out_t(const unsigned char *bytes, cassie_sm_out_t *bus);

#ifdef __cplusplus
}
#endif
#endif // CASSIE_SM_T_H

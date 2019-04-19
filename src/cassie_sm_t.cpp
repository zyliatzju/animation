#include "animation/cassie_sm_t.hpp"
#include <stddef.h>
#include <string.h>



void pack_cassie_sm_in_t(const cassie_sm_in_t *bus, unsigned char bytes[4])
{
  unsigned char y[4];
  int x[1];
  x[0] = (int)bus->motion_id;

  memcpy((void *)&y[0], (void *)&x[0], (unsigned int)((size_t)4 * sizeof
          (unsigned char)));

  for (int i0 = 0; i0 < 4; i0++) {
    bytes[i0] = y[i0];
  }

}


void unpack_cassie_sm_out_t(const unsigned char bytes[8], cassie_sm_out_t
  *bus)
{
  short y[3];
  unsigned char x[6];
  for (int i=0; i<6; i++){
    x[i] = bytes[i];
  }

  memcpy((void *)&y[0], (void *)&x[0], (unsigned int)((size_t)3 * sizeof(short)));
  bus->track_ani = y[0];
  bus->in_ani = y[1];
  bus->ani_finished = y[2];
}

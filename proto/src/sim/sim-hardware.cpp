/* Code to manage the interface between the simulator and the kernel.
Copyright (C) 2005-2008, Jonathan Bachrach, Jacob Beal, and contributors 
listed in the AUTHORS file in the MIT Proto distribution's top directory.

This file is part of MIT Proto, and is distributed under the terms of
the GNU General Public License, with a linking exception, as described
in the file LICENSE in the MIT Proto distribution's top directory. */

#include "config.h"
#include "spatialcomputer.h"

/*****************************************************************************
 *  SIMULATED HARDWARE                                                       *
 *****************************************************************************/
// globals managed by set_vm_context
SimulatedHardware* hardware=NULL;
Device* device=NULL;

SimulatedHardware::SimulatedHardware() {
  for(int i=0;i<NUM_HARDWARE_FNS;i++) patch_table[i]=&base;
}

void SimulatedHardware::patch(HardwarePatch* p, HardwareFunction fn) {
  patch_table[fn]=p;
}

// The kernel lives in C on a single device
// This function sets the globals that connect it to a simulated device.
void SimulatedHardware::set_vm_context(Device* d) {
  hardware = this;
  device = d;
  machine = d->vm;
  is_debugging_val = is_kernel_debug && d->debug();
  is_tracing_val = is_kernel_trace && d->debug();
  is_script_debug_val = is_kernel_debug_script && d->debug();
}

/*****************************************************************************
 *  DEBUGGING INFORMATION                                                    *
 *****************************************************************************/
// The kernel uses these to decide what information to post
// debug_id specifies which node is being debugged.
int debug_id = -1; // -1 means all: we'll set it there and modulate is_X
int is_debugging_val = 0;
int is_tracing_val = 0;
int is_script_debug_val = 0;


/*****************************************************************************
 *  KERNEL HARDWARE CALLOUTS                                                 *
 *****************************************************************************/

void reinitHardware(void) { return; }

void mov (VEC_VAL *val)
{ hardware->patch_table[MOV_FN]->mov(val); }
void flex (NUM_VAL val) 
{ hardware->patch_table[FLEX_FN]->flex(val); }
NUM_VAL cam_get (int k) 
{ return hardware->patch_table[CAM_GET_FN]->cam_get(k); }
NUM_VAL radius_get (VOID)
{ return hardware->patch_table[RADIUS_GET_FN]->radius_get(); }
NUM_VAL radius_set (NUM_VAL val) 
{ return hardware->patch_table[RADIUS_SET_FN]->radius_set(val); }
void die (NUM_VAL val) 
{ hardware->patch_table[DIE_FN]->die(val); }
void clone_machine (NUM_VAL val) 
{ hardware->patch_table[CLONE_MACHINE_FN]->clone_machine(val); }
void set_r_led (NUM_VAL val) 
{ hardware->patch_table[SET_R_LED_FN]->set_r_led(val); }
void set_g_led (NUM_VAL val) 
{ hardware->patch_table[SET_G_LED_FN]->set_g_led(val); }
void set_b_led (NUM_VAL val) 
{ hardware->patch_table[SET_B_LED_FN]->set_b_led(val); }
void set_probe (DATA* d, uint8_t p) 
{ hardware->patch_table[SET_PROBE_FN]->set_probe(d,p); }

void set_dt (NUM_VAL dt)
{ hardware->patch_table[SET_DT_FN]->set_dt(dt);}

void set_is_folding (BOOL val, int k) 
{ hardware->patch_table[SET_IS_FOLDING_FN]->set_is_folding(val,k); }
BOOL read_fold_complete (int val) 
{ return hardware->patch_table[READ_FOLD_COMPLETE_FN]->
    read_fold_complete(val); }

NUM_VAL set_channel (NUM_VAL diffusion, int k) 
{ return hardware->patch_table[SET_CHANNEL_FN]->set_channel(diffusion,k); }
NUM_VAL read_channel (int k) 
{ return hardware->patch_table[READ_CHANNEL_FN]->read_channel(k); }
NUM_VAL drip_channel (NUM_VAL val, int k) 
{ return hardware->patch_table[DRIP_CHANNEL_FN]->drip_channel(val,k); }
VEC_VAL *grad_channel (int k) 
{ return hardware->patch_table[GRAD_CHANNEL_FN]->grad_channel(k); }
NUM_VAL read_radio_range (VOID) 
{ return hardware->patch_table[READ_RADIO_RANGE_FN]->read_radio_range(); }
NUM_VAL read_light_sensor (VOID) 
{ return hardware->patch_table[READ_LIGHT_SENSOR_FN]->read_light_sensor(); }
NUM_VAL read_microphone (VOID) 
{ return hardware->patch_table[READ_MICROPHONE_FN]->read_microphone(); }
NUM_VAL read_temp (VOID) 
{ return hardware->patch_table[READ_TEMP_FN]->read_temp(); }
NUM_VAL read_short (VOID) 
{ return hardware->patch_table[READ_SHORT_FN]->read_short(); }
NUM_VAL read_sensor (uint8_t n) 
{ return hardware->patch_table[READ_SENSOR_FN]->read_sensor(n); }
VEC_VAL *read_coord_sensor (VOID) 
{ return hardware->patch_table[READ_COORD_SENSOR_FN]->read_coord_sensor(); }
VEC_VAL *read_mouse_sensor (VOID) 
{ return hardware->patch_table[READ_MOUSE_SENSOR_FN]->read_mouse_sensor(); }
VEC_VAL *read_ranger (VOID) 
{ return hardware->patch_table[READ_RANGER_FN]->read_ranger(); }
NUM_VAL read_bearing (VOID) 
{ return hardware->patch_table[READ_BEARING_FN]->read_bearing(); }
NUM_VAL read_speed (VOID) 
{ return hardware->patch_table[READ_SPEED_FN]->read_speed(); }
NUM_VAL read_bump (VOID) 
{ return hardware->patch_table[READ_BUMP_FN]->read_bump(); }
NUM_VAL read_button (uint8_t n) 
{ return hardware->patch_table[READ_BUTTON_FN]->read_button(n); }
NUM_VAL read_slider (uint8_t ikey, uint8_t dkey, NUM_VAL init, NUM_VAL incr, 
                     NUM_VAL min, NUM_VAL max) {
  return hardware->patch_table[READ_SLIDER_FN]->
    read_slider(ikey,dkey,init,incr,min,max);
}
void set_speak (NUM_VAL period) 
{ hardware->patch_table[SET_SPEAK_FN]->set_speak(period); }

int radio_send_export (uint8_t version, uint8_t timeout, uint8_t n, 
                       uint8_t len, COM_DATA *buf) {
  return hardware->patch_table[RADIO_SEND_EXPORT_FN]->
    radio_send_export(version,timeout,n,len,buf); 
}
int radio_send_script_pkt (uint8_t version, uint16_t n, uint8_t pkt_num, 
                           uint8_t *script) {
return hardware->patch_table[RADIO_SEND_SCRIPT_PKT_FN]->
  radio_send_script_pkt(version,n,pkt_num,script);
}
int radio_send_digest (uint8_t version, uint16_t script_len, uint8_t *digest) {
  return hardware->patch_table[RADIO_SEND_DIGEST_FN]->
    radio_send_digest(version,script_len,digest);
}

extern void my_platform_operation(uint8_t op); 
void platform_operation(uint8_t op) { my_platform_operation(op); }

/*****************************************************************************
 *  MEMORY MANAGEMENT                                                        *
 *****************************************************************************/
// the simulator grants only a fixed-size block of memory to each machine
int MAX_MEM_SIZE = 4*4096;
uint8_t* MEM_CONS(MACHINE *m) {
  m->memlen = MAX_MEM_SIZE;
  m->membuf = (uint8_t*)calloc(m->memlen,1);
  if(m->membuf==NULL) uerror("Malloc failed for membuf!");
  m->saved_memptr = m->memptr = 0;
  return m->membuf;
}
void MEM_GROW (MACHINE *m) { uerror("OUT OF MEM"); }

MACHINE* allocate_machine() {
  MACHINE* vm = (MACHINE*)calloc(1,sizeof(MACHINE));
  MEM_CONS(vm);
  return vm;
}
void deallocate_machine(MACHINE** vm) {
  FREE(&(*vm)->membuf); 
  FREE(vm);
}

/*****************************************************************************
 *  PRETTY-PRINTING                                                          *
 *****************************************************************************/
void post_data_to2 (char *str, DATA *d, int verbosity) {
  char buf[100];
  if (d->is_dead) { // Note: we should never see "DEAD" markers
    sprintf(buf, "(DEAD "); strcat(str, buf); 
  }
  switch (d->tag) {
  case NUM_TAG: {
    sprintf(buf, "%.2f", NUM_GET(d)); strcat(str, buf); break; }
  case FUN_TAG: { // Note: a function is just an ID
    sprintf(buf, "F%d", FUN_GET(d)); strcat(str, buf); break; }
  case VEC_TAG: {
    int i;
    VEC_VAL *v = VEC_GET(d);
    if(verbosity>0) strcat(str, "[");
    for (i = 0; i < v->n; i++) {
      if (i != 0) strcat(str, " ");
      post_data_to2(str, &v->elts[i], verbosity);
    }
    if(verbosity>0) strcat(str, "]");
    break; }
  }
  if (d->is_dead) {
    sprintf(buf, ")"); strcat(str, buf); 
  }
}

void post_stripped_data_to (char *str, DATA *d) {
  strcpy(str, "");
  post_data_to2(str, d, 0);
}

void post_data_to (char *str, DATA *d) {
  strcpy(str, "");
  post_data_to2(str, d, 1);
}

void post_data (DATA *d) {
  char buf[256];
  post_data_to(buf, d);
  post(buf);
}
/**
 * File:   cond_var_null.c
 * Author: AWTK Develop Team
 * Brief:  cond_var do nothing
 *
 * Copyright (c) 2018 - 2020  Guangzhou ZHIYUAN Electronics Co.,Ltd.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * License file for more details.
 *
 */

/**
 * History:
 * ================================================================
 * 2019-05-11 Li XianJing <xianjimli@hotmail.com> created
 *
 */

#include "tkc/mem.h"
#include "tkc/cond_var.h"

struct _tk_cond_var_t {
  uint32_t none;
};

static const tk_cond_var_t s_cond_var_null;

tk_cond_var_t* tk_cond_var_create(void) {
  return (tk_cond_var_t*)&s_cond_var_null;
}

ret_t tk_cond_var_wait(tk_cond_var_t* cond_var, uint32_t timeout_ms) {
  return RET_OK;
}

ret_t tk_cond_var_awake(tk_cond_var_t* cond_var) {
  return RET_OK;
}

ret_t tk_cond_var_destroy(tk_cond_var_t* cond_var) {
  return RET_OK;
}

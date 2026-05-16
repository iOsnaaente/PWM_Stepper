/**
 * @file    comm_base.h
 * @brief   Interface abstrata minima para modulos de comunicacao.
 * @author  Bruno G. F. Sampaio
 * @date    Criado em 12 de Outubro de 2025
 */

#pragma once

#include "comm_base_utils.h"

class CommBase {

  private:
    const char   *description;
    CommRet_t     last_error;
    CommState_t   state;

    uint64_t      msg_counter;

    CommType_t    type_;
    CommAddr_t    addr_;

  public:

    CommMessage_t last_message;
    uint64_t      msg_received;
    uint64_t      msg_sent;
    bool          connected;

    CommBase( const char* description = nullptr )
      : description(description),
        last_error(COMM_RET_OK),
        state(COMM_STATE_UNINITIALIZED),
        msg_counter(0),
        msg_received(0),
        msg_sent(0),
        connected(false)
    {
      if (this->description == nullptr) {
        this->description = "CommunicationBase";
      }
    }

    virtual CommRet_t init() = 0;
    virtual CommRet_t deinit() = 0;
    virtual CommRet_t start() = 0;
    virtual CommRet_t stop() = 0;

    virtual CommRet_t send( const CommMessage_t &msg, uint32_t timeout_ms = 0 ) = 0;

    virtual CommRet_t send_to( const CommMessage_t &msg, const CommAddr_t &addr, uint32_t timeout_ms = 0) {
      (void)timeout_ms;
      (void)addr;
      (void)msg;
      return COMM_RET_NOT_IMPLEMENTED;
    }

    virtual CommRet_t receive( CommMessage_t &out_msg, uint32_t timeout_ms = 0 ) = 0;

    virtual CommRet_t receive_from( CommMessage_t &out_msg, CommAddr_t &addr, uint32_t timeout = 0 ){
      (void)out_msg;
      (void)addr;
      (void)timeout;
      return COMM_RET_NOT_IMPLEMENTED;
    }

    virtual size_t available() const {
      return 0;
    }

    virtual CommRet_t peek(CommMessage_t &out_msg) {
      (void)out_msg;
      return COMM_RET_NOT_IMPLEMENTED;
    }

    virtual uint32_t get_num_messages() const {
      return this->msg_counter;
    }

    virtual CommRet_t flush() {
      return COMM_RET_NOT_IMPLEMENTED;
    }

    virtual CommRet_t get_last_error() const {
      return this->last_error;
    }

    virtual CommState_t get_state() const {
      return this->state;
    }

    virtual const char* get_description() const {
      return this->description;
    }

    virtual ~CommBase() = default;


  protected:
    inline void incr_msg_counter() {
      this->msg_counter++;
    }
    inline void reset_msg_counter() {
      this->msg_counter = 0;
    }
    inline void set_last_error(CommRet_t err ) {
      this->last_error = err;
    }
    inline void set_state(CommState_t state ) {
      this->state = state;
    }
};

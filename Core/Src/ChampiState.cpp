/*
 * ChampiState.cpp
 *
 *  Created on: Mar 3, 2024
 *      Author: arusso
 */

#include <ChampiState.h>

ChampiState::ChampiState() = default;

ChampiState::ChampiState(ChampiCan *champi_can_interface, uint32_t tx_period_ms) {
    status_msg_ = msgs_can_Status_init_zero;
    status_msg_.has_status = true;
    status_msg_.has_error = true;
    champi_can_interface_ = champi_can_interface;
    tx_period_ms_ = tx_period_ms;
    last_tx_time_ms_ = 0;
}

ChampiState::~ChampiState() = default;


void ChampiState::report_status(msgs_can_Status_StatusType status, msgs_can_Status_ErrorType error) {

    // If it's different from the current status, we sent to the CAN right away (for not loosing time to report error)
    if(status_msg_.status != status || status_msg_.error != error) {
        status_msg_.status = status;
        status_msg_.error = error;

        send_status();

        // TODO on pourra ajouter le logging ici plus tard !
    }
}

/**
 * Send the status message to the CAN bus
 *
 * Ici, on ne check pas les erreurs d'envoi. Ca permet d'essayer d'envoyer le status si on est dans l'état d'erreur.
 * De toute façon, s'il y a un problème d'envoi, on envoie assez de messages ailleurs pour s'en rendre compte
 * vite.
 */
void ChampiState::send_status() {

    // Init stream TODO can we do that only once ?
    pb_ostream_t stream = pb_ostream_from_buffer(tx_buffer_, sizeof(tx_buffer_));

    // Encode message
    pb_encode(&stream, msgs_can_Status_fields, &status_msg_);
    size_t message_length = stream.bytes_written;

    // Send message
    champi_can_interface_->send_msg(CAN_ID_IMU_STATUS, tx_buffer_, message_length); // TODO make the ID configurable
}

void ChampiState::spin_once() {

    // 1st time: Init and return
    if(last_tx_time_ms_ == 0) {
        last_tx_time_ms_ = HAL_GetTick();
        return;
    }

    // Send status if needed
    uint32_t current_time_ms = HAL_GetTick();
    if(current_time_ms - last_tx_time_ms_ > tx_period_ms_) {
        send_status();
        last_tx_time_ms_ = current_time_ms;
    }
}

#include "odriveS1.hpp"

// Add function definitions here!
ODRIVES1::ODRIVES1 (FDCAN_HandleTypeDef* fdcanhandle) : _can(fdcanhandle) {
	// TODO: Add Error Handling
	// Configure Filter
	this->odriveCanFilter.IdType = FDCAN_STANDARD_ID;
	this->odriveCanFilter.FilterIndex = 0;
	// Set our filter to mask so it uses ID1 as a value and ID2 as mask
	this->odriveCanFilter.FilterType = FDCAN_FILTER_MASK;
	this->odriveCanFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	// Assume node_id of Odrive is 0
	this->odriveCanFilter.FilterID1 = 0x000;
	// Mask first five bits (16 bit mask, 11 bit CAN address)
	this->odriveCanFilter.FilterID2 = 0b11111 << 10;
	this->odriveCanFilter.RxBufferIndex = 0;

	HAL_FDCAN_ConfigFilter(this->_can, &this->odriveCanFilter);

	HAL_FDCAN_Start(this->_can);
	HAL_FDCAN_ActivateNotification(this->_can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

HAL_StatusTypeDef ODRIVES1::sendGetMsgCAN(uint32_t identifier) {
	FDCAN_TxHeaderTypeDef txHeader;
	txHeader.Identifier = identifier;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_REMOTE_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0x00; // Ignore because FDCAN_NO_TX_EVENTS

	// Add bytes to queue to be sent
	HAL_FDCAN_AddMessageToTxFifoQ(this->_can, &txHeader, this->odriveTxBuffer);

	// Wait for a message to come in
	while(HAL_FDCAN_GetRxFifoFillLevel(this->_can, FDCAN_RX_FIFO0) < 1) {
	}

	// Retrieve elements from FIFO queue
	if (HAL_FDCAN_GetRxMessage(this->_can, FDCAN_RX_FIFO0, &this->odriveCanRxHeader, this->odriveRxBuffer) != HAL_OK) {
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef ODRIVES1::sendSetMsgCAN(uint32_t identifier) {
	FDCAN_TxHeaderTypeDef txHeader;
	txHeader.Identifier = identifier;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0x00; // Ignore because FDCAN_NO_TX_EVENTS

	// Add bytes to queue to be sent
	if (HAL_FDCAN_AddMessageToTxFifoQ(this->_can, &txHeader, this->odriveTxBuffer) != HAL_OK) {
		return HAL_ERROR;
	}

	return HAL_OK;
}

#pragma once

#include "Arduino.h"
#include <libmaple/usart.h>
#include <HardwareSerial.h>
#include "DataPacker2.h"
#include "Schedule.h"

#include "Logger.h"

#ifndef UART_PACKET_SIZE
#define UART_PACKET_SIZE	32
#endif

#ifndef UART_PACKET_MIN_INTERVAL
#define UART_PACKET_MIN_INTERVAL 10000
#endif

typedef DataPacker2<UART_PACKET_SIZE> UartData;

class AsyncUart {
public:
  AsyncUart(HardwareSerial * port, const uint8_t address)
	: m_port(port),
	m_dev(port->c_dev()),
	m_address(address) {
  }

  void begin(const uint32_t & baudrate, const uint32_t timeout = 5) {
	m_port->begin(baudrate);
	m_port->setTimeout(timeout);
  }

  void write(const UartData & data) {
	m_busy = true;
	m_out_data = data;
	m_out_data.putCRC();
	usart_reset_tx(m_dev);
	usart_tx(m_dev, m_out_data.getBuffer(), m_out_data.size());
  }

  UartData read() {
	m_available = false;
	m_rx_error = false;
	return m_in_data;
  }

  void update() {
	const uint32_t current_us = micros();
	
	SCHEDULER_GUARD(current_us, m_last_rx_us);
	SCHEDULER_GUARD(current_us, m_last_tx_us);

	if (isTxBufferEmpty()
	  && current_us > m_last_tx_us + 2000)
	  m_busy = false;

	// clean buffer if trash received
	if (current_us < m_last_rx_us + UART_PACKET_MIN_INTERVAL)
	  usart_reset_rx(m_dev);

	if (m_port->available() >= m_in_data.size() - 1) {
	  m_last_rx_us = current_us;
	  m_in_data.clear();
	
	  m_port->readBytes(m_in_data.getBuffer(), m_in_data.size());

	  //Serial.println(m_in_data.dump());
	  m_rx_error = !m_in_data.checkCRC();

	  if (!m_rx_error) {
		uint8_t initMap[] = { 1, 1 };
		m_in_data.setSizeMap(initMap, sizeof(initMap));
		const auto from_addr = m_in_data.get<uint8_t>(0);
		const auto to_addr = m_in_data.get<uint8_t>(1);
		//DEBUGF("msg from %d to %d", from_addr, to_addr);

		if (to_addr == m_address)
		  m_available = true;
	  }
	  else {
		ERROR("CRC error");
	  }
	}
  }

  bool available() {
	return m_available;
  }

  bool rx_error() {
	const auto e = m_rx_error;
	m_rx_error = false;
	return e;
  }

  bool busy() {
	return m_busy;
  }
private:

  bool isTxBufferEmpty() {
	constexpr auto _USART_SR_TC_BIT_ = 6;
	if (rb_is_empty(m_dev->wb)								// wait for TX buffer empty
	  && ((m_dev->regs->SR) & (1 << _USART_SR_TC_BIT_)))	// wait for TC (Transmission Complete) flag set
															// at least 2ms after buffer is empty
	  return true;
	return false;
  }

  HardwareSerial * const m_port;
  usart_dev * const m_dev;
  const uint8_t m_address;

  UartData m_in_data, m_out_data;
  bool m_rx_error{ false };
  bool m_available{ false };
  bool m_busy{ false };

  uint32_t m_last_tx_us{ 0 };
  uint32_t m_last_rx_us{ 0 };
};


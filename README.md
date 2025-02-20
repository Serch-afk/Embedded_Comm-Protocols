# Embedded_Comm-Protocols
Practicas desarrolladas para el curso de Sistemas De Comunicación Para Sistemas Embebidos E Internet De Las Cosas

Practica 1:
Descripción del API
>>> Funciones Principales
* ETHCOMM_vInit(): Inicializa la interfaz Ethernet.
  
* ETHCOMM_vMsgSend(uint8_t* pu8Buffer, uint32_t u32DataLength): Envía un mensaje a través de Ethernet.
  pu8Buffer: Puntero al buffer de datos a enviar.
  u32DataLength: Cantidad de bytes a enviar.
  
* ETHCOMM_u16MsgReceive(uint8_t* pu8MsgBuffer): Recibe un mensaje a través de Ethernet.
  pu8MsgBuffer: Puntero al buffer donde se almacenará la información recibida.

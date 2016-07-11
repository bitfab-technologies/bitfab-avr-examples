#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#define SPI_SS_bm             0x10 /*!< \brief Bit mask for the SS pin. */
#define SPI_MOSI_bm           0x20 /*!< \brief Bit mask for the MOSI pin. */
#define SPI_MISO_bm           0x40 /*!< \brief Bit mask for the MISO pin. */
#define SPI_SCK_bm            0x80 /*!< \brief Bit mask for the SCK pin. */

#define SPI_OK              0     /*!< \brief The transmission completed successfully. */
#define SPI_INTERRUPTED     1     /*!< \brief The transmission was interrupted by another master. */
#define SPI_BUSY            2     /*!< \brief The SPI module is busy with another transmission. */


/*! \brief SPI data packet struct. */
typedef struct SPI_DataPacket
{
  PORT_t *ssPort;                     /*!< \brief Pointer to SS port. */
  uint8_t ssPinMask;                  /*!< \brief SS pin mask. */
  const uint8_t *transmitData;        /*!< \brief Pointer to data to transmit. */
  volatile uint8_t *receiveData;      /*!< \brief Pointer to where to store received data. */
  volatile uint8_t bytesToTransceive; /*!< \brief Number of bytes to transfer. */
  volatile uint8_t bytesTransceived;  /*!< \brief Number of bytes transfered. */
  volatile bool complete;             /*!< \brief Complete flag. */
} SPI_DataPacket_t;


/*! \brief SPI master struct. Holds pointer to SPI module, buffers and necessary varibles. */
typedef struct SPI_Master
{
  SPI_t *module;                /*!< \brief Pointer to what module to use. */
  PORT_t *port;                 /*!< \brief Pointer to port. */
  bool interrupted;             /*!< \brief True if interrupted by other master (SS pulled low). */
  SPI_DataPacket_t *dataPacket; /*!< \brief Holds transceive data. */
} SPI_Master_t;


/*! \brief SPI slave struct. Holds pointers to SPI module and used port. */
typedef struct SPI_Slave
{
  SPI_t *module;      /*!< \brief Pointer to what module to use. */
  PORT_t *port;       /*!< \brief Pointer to port. */
} SPI_Slave_t;


/* Definitions of macros. */


/*! \brief Checks if transmission is complete.
 *
 *  \param _spi     Pointer to SPI_Master_t struct instance.
 *
 *  \return         The current status of the transmission.
 *  \retval true    The transmission is complete.
 *  \retval false   The transmission is in progress.
 */
#define SPI_MasterInterruptTransmissionComplete(_spi) ( (_spi)->dataPacket->complete )



/*! \brief Pulls SPI SS line(s) low in order to address the slave devices.
 *
 *  \param _port         Pointer to the I/O port where the SS pins are located.
 *  \param _pinBM        A bitmask selecting the pins to pull low.
 *
 *  \retval NA
 */
#define SPI_MasterSSLow(_port, _pinBM) ( (_port)->OUTCLR = (_pinBM) )



/*! \brief Releases SPI SS line(s).
 *
 *  \param _port         Pointer to the I/O port where the SS pins are located.
 *  \param _pinBM        A bitmask selecting the pins to release.
 *
 *  \retval NA
 */
#define SPI_MasterSSHigh(_port, _pinBM) ( (_port)->OUTSET = (_pinBM) )



/*! \brief Write data byte to the SPI shift register.
 *
 *  \param _spi        Pointer to SPI_Slave_t struct instance.
 *  \param _data       The data to write to the shift register.
 */
#define SPI_SlaveWriteByte(_spi, _data) ( (_spi)->module->DATA = (_data) )



/*! \brief Read received data byte.
 *
 *  \param _spi       Pointer to SPI_Slave_t struct instance.
 *
 *  \return           The received data.
 */
#define SPI_SlaveReadByte(_spi) ( (_spi)->module->DATA )



/*! \brief Check if new data is available.
 *
 *  \param _spi       Pointer to SPI_Slave_t struct instance.
 *
 *  \return           True if data available, false if not.
 */
#define SPI_SlaveDataAvailable(_spi) ( (_spi)->module->STATUS & SPI_IF_bm )


/* Prototype functions. Documentation found in source file */

void SPI_MasterInit(SPI_Master_t *spi,
                    SPI_t *module,
                    PORT_t *port,
          bool lsbFirst,
                    SPI_MODE_t mode,
                    SPI_INTLVL_t intLevel,
                    bool clk2x,
                    SPI_PRESCALER_t clockDivision);

void SPI_SlaveInit(SPI_Slave_t *spi,
                   SPI_t *module,
                   PORT_t *port,
                   bool lsbFirst,
                   SPI_MODE_t mode,
                   SPI_INTLVL_t intLevel);

void SPI_MasterCreateDataPacket(SPI_DataPacket_t *dataPacket,
                                const uint8_t *transmitData,
                                uint8_t *receiveData,
                                uint8_t bytesToTransceive,
                                PORT_t *ssPort,
                                uint8_t ssPinMask);

void SPI_MasterInterruptHandler(SPI_Master_t *spi);

uint8_t SPI_MasterInterruptTransceivePacket(SPI_Master_t *spi,
                                            SPI_DataPacket_t *dataPacket);

uint8_t SPI_MasterTransceiveByte(SPI_Master_t *spi, uint8_t TXdata);

bool SPI_MasterTransceivePacket(SPI_Master_t *spi,
                                SPI_DataPacket_t *dataPacket);

#endif

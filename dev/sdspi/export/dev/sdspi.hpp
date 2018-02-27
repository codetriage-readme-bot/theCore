/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef DEV_SDSPI_HPP
#define DEV_SDSPI_HPP

#include <algorithm>
#include <array>
#include <string.h>

#include <ecl/iostream.hpp>
#include <ecl/endian.hpp>
#include <ecl/types.h>

namespace ecl
{

// TODO: mention about 8 additional clocks before each command!!!
template<class spi_dev, class gpio_cs>
class sd_spi
{
public:
    // Lazy initialization, -1 if error. 0 otherwise
    static err init();
    // -1 if error, 0 otherwise
    static err deinit();

    // < count if error, count otherwise
    static err write(const uint8_t *data, size_t &count);
    // -1 if error, [0, count] otherwise
    static err read(uint8_t *data, size_t &count);
    // Flushes buffered data
    static err flush();

    // TODO: extend it with SEEK_CUR and SEEK_END
    // Seeks to the given position, in bytes
    static err seek(off_t offset /*, SEEK_SET */);

    // Tell current position
    // -1 if error, valid offset otherwise
    static err tell(off_t &offt);

    // Returns a length of a block
    static constexpr size_t get_block_length();

    // Seek flags
    enum class seekdir
    {
        beg = 0,
        cur = 1,
        end = 2
    };

private:
    // Card types
    enum class sd_type
    {
        hc,   // The card is high capacity
        sc,   // The card is standart capacity
        unkn, // Unknown card type
    };

    // R1 response
    struct R1
    {
        R1()      :response{0} {}
        uint8_t    response;

        bool ok() const
        {
            // If anything set besides idle flags it is an error
            return received() && !(response & ~idle_state);
        }

        bool received() const
        {
            // Reserved bit indicates a start of R1 response
            return !(response & reserved_bit);
        }

        // Flags that can be found in response
        static constexpr uint8_t idle_state    = 0x01;
        static constexpr uint8_t erase_reset   = 0x02;
        static constexpr uint8_t illegal_cmd   = 0x04;
        static constexpr uint8_t com_crc_err   = 0x08;
        static constexpr uint8_t erase_seq_err = 0x10;
        static constexpr uint8_t addr_err      = 0x20;
        static constexpr uint8_t param_error   = 0x40;
        // Must be set to 0 in response
        static constexpr uint8_t reserved_bit  = 0x80;
    };

    // R3 response
    struct R3
    {
        R3()      :r1{}, OCR{0} {}

        R1         r1;
        uint32_t   OCR;
    };

    // Are the same
    using R7 = R3;
    using argument = std::array< uint8_t, 4 >;

    // http://elm-chan.org/docs/mmc/mmc_e.html
    // SD SPI command set ------------------------------------------------------

    // GO_IDLE_STATE            - Software reset.
    static err CMD0(R1 &r);

    // SEND_OP_COND             - Initiate initialization process.
    static err CMD1(R1 &r);

    // APP_SEND_OP_COND         - For only SDC. Initiate initialization process.
    static err ACMD41(R1 &r, bool HCS);

    // SEND_IF_COND             - For only SDC V2. Check voltage range.
    static err CMD8(R7 &r);

    // SEND_CSD                 - Read CSD register.
    static err CMD9(R1 &r, uint8_t *buf, size_t size);

    // SEND_CID                 - Read CID register.
    static err CMD10(R1 &r);

    // STOP_TRANSMISSION        - Stop to read data.
    // int CMD12(R1 &r); R1b is required here

    // SET_BLOCKLEN             - Change R/W block size.
    static err CMD16(R1 &r);

    // READ_SINGLE_BLOCK        - Read a block.
    static err CMD17(R1 &r, uint32_t address);

    // READ_MULTIPLE_BLOCK      - Read multiple blocks.
    static err CMD18(R1 &r, uint32_t address, uint8_t *buf, size_t size);

    // SET_BLOCK_COUNT          - For only MMC. Define number of blocks to transfer
    //                            with next multi-block read/write command.
    static err CMD23(R1 &r, uint16_t block_count);

    // SET_WR_BLOCK_ERASE_COUNT - For only SDC. Define number of blocks to pre-erase
    //                              with next multi-block write command.
    static err ACMD23(R1 &r, uint16_t block_count);

    // WRITE_BLOCK              - Write a block.
    static err CMD24(R1 &r, uint32_t address);

    // WRITE_MULTIPLE_BLOCK     - Write multiple blocks.
    static err CMD25(R1 &r, uint32_t address, uint8_t *buf, size_t size);

    // APP_CMD                  - Leading command of ACMD<n> command.
    static err CMD55(R1 &r);

    // READ_OCR                 - Read OCR.
    static err CMD58(R3 &r);

    // Sends CMD
    template< typename R >
    static err send_CMD(R &resp, uint8_t CMD_idx, const argument &arg, uint8_t crc = 0);

    // Sends >= 47 empty clocks to initialize the device
    static err send_init();
    static err open_card();
    static err software_reset();
    static err check_conditions();
    static err init_process();
    static err check_OCR(sd_type &type);
    static err obtain_card_info();
    static err set_block_length();
    static err populate_block(size_t new_block);
    static err flush_block();
    static err traverse_data(size_t count,
        const std::function<void(size_t data_offt, size_t blk_offt, size_t amount)>& fn);

    // Useful abstractions
    static err receive_response(R1 &r);
    static err receive_response(R3 &r);
    //err receive_response(R1_read &r);

    static err receive_data(uint8_t *buf, size_t size);
    static err send_data(const uint8_t *buf, size_t size);

    // Transport layer TODO: merge these three
    static err spi_send(const uint8_t *buf, size_t &size);
    static err spi_receive(uint8_t *buf, size_t &size);
    static err spi_send_dummy(size_t &size);


    // POD-type, allocated as a static object.
    // All fields initialized to 0 during static object initialization
    static struct
    {
        bool          inited;     // Init flag
        bool          hc;         // High Capacity flag
        off_t         offt;       // Current offset in units of bytes

        // POD-type, allocated as an subobject of a static object.
        // All fields initialized to 0 during static object initialization
        struct
        {
            // Block length is given without respect to the card type.
            // If card is Standart Capacity then block length will be set to 512,
            // ( see set_block_length() ).
            // If card is High Capacity then block length is fixed to 512 by design.
            static constexpr size_t block_len = 512;

            uint8_t buf[block_len];       // The block itself
            size_t  origin;               // The offset from which block was obtained
            bool    mint;                 // True if there were no writes in buffer
        } block;    // Buffer containing last read\write block
    } m_ctx;
};

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::init()
{
    err rc;
    ecl_assert(!m_ctx.inited);
    m_ctx.block.mint = true;

    rc = spi_dev::init();
    if (is_error(rc)) {
        return rc;
    }

    spi_dev::lock();
    gpio_cs::set();
    spi_dev::unlock();

    spi_dev::lock();
    rc = send_init();

    if (is_ok(rc)) {
        gpio_cs::reset();
        rc = open_card();
        gpio_cs::set();
    }

    spi_dev::unlock();

    if (is_ok(rc)) {
        m_ctx.m_inited = true;
    }

    return rc;
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::close()
{
    // TODO
    return err::ok;
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::write(const uint8_t *data, size_t &count)
{
    ecl_assert(m_ctx.inited);

    err rc;
    auto fn = [data](size_t data_offt, size_t blk_offt, size_t to_copy) {
        memcpy(m_ctx.block.buf + blk_offt, data + data_offt, to_copy);
        m_ctx.block.mint = false;
    };

    return traverse_data(count, fn);
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::read(uint8_t *data, size_t &count)
{
    ecl_assert(m_ctx.inited);

    err rc;
    auto fn = [data, this](size_t data_offt, size_t blk_offt, size_t to_copy) {
        memcpy(data + data_offt, m_ctx.block.buf + blk_offt , to_copy);
    };

    return traverse_data(count, fn);
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::flush()
{
    ecl_assert(m_ctx.inited);

    int rc;

    spi_dev::lock();
    gpio_cs::reset();

    rc = flush_block();

    gpio_cs::set();
    spi_dev::unlock();

    return rc;
}

// TODO: change off_t to int
template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::seek(off_t offset)
{
    ecl_assert(m_ctx.inited);
    m_ctx.offt = offset;
    return err::ok;
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::tell(off_t offt)
{
    ecl_assert(m_ctx.inited);
    return m_ctx.offt;
}

template<class spi_dev, class gpio_cs>
constexpr size_t sd_spi<spi_dev, gpio_cs>::get_block_length()
{
    return m_ctx.block::block_len;
}

// Private methods -------------------------------------------------------------

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::spi_send(const uint8_t *buf, size_t &size)
{
    spi_dev::set_buffers(buf, nullptr, size);
    // TODO: verify that all data was transferred
    auto rc = spi_dev::xfer(&size, nullptr);

    // Do not propagate exact bus error code to user.
    if (is_error(rc)) {
        rc = err::io;
    }

    return rc;
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::spi_receive(uint8_t *buf, size_t &size)
{
    spi_dev::set_buffers(nullptr, buf, size);
    // TODO: verify that all data was transferred
    auto rc = spi_dev::xfer(nullptr, &size);

    // Do not propagate exact bus error code to user.
    if (is_error(rc)) {
        rc = err::io;
    }

    return rc;
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::spi_send_dummy(size_t &size)
{
    spi_dev::set_buffers(size);
    // TODO: verify that all data was transferred
    auto rc = spi_dev::xfer(&size, nullptr);

    // Do not propagate exact bus error code to user.
    if (is_error(rc)) {
        rc = err::io;
    }

    return rc;
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::send_init()
{
    // Initialise card with >= 74 clocks on start
    return spi_send_dummy(80);
}

template<class spi_dev, class gpio_cs>
template<typename R>
err sd_spi<spi_dev, gpio_cs>::send_CMD(R &resp, uint8_t CMD_idx, const argument &arg, uint8_t crc)
{
    err rc;

    CMD_idx &= 0x3f; // First two bits are reserved TODO: comment
    CMD_idx |= 0x40;

    crc |= 0x1; // EOT flag

    // Init a transaction
    spi_send_dummy(1);

    // Command body
    const uint8_t to_send[] =
        { CMD_idx, arg[0], arg[1], arg[2], arg[3], crc };

    // Send HCS
    rc = spi_send(to_send, sizeof(to_send));
    if (is_error(rc)) {
        return rc;
    }

    // Retrieve a result
    return receive_response(resp);
}

//------------------------------------------------------------------------------

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::receive_response(R1 &r)
{
    uint8_t tries = 8;
    err rc

    do {
        rc = spi_receive(&r.response, 1);
        if (is_error) {
            return rc;
        }
    } while (!r.received() && --tries);

    if (!r.received()) {
        ecl::cout << "R1 response wait expired." << ecl::endl;
        rc = err::generic; // TODO: better error code
    } else if (!r.ok()) {
        ecl::cout << "R1 error(s) received:" << ecl::endl;

        if (r.response & R1::erase_reset) {
            ecl::cout << "Erase reset error" << ecl::endl;
        }
        if (r.response & R1::illegal_cmd) {
            ecl::cout << "Illegal command error" << ecl::endl;
        }
        if (r.response & R1::com_crc_err) {
            ecl::cout << "CRC error" << ecl::endl;
        }
        if (r.response & R1::erase_seq_err) {
            ecl::cout << "Erase sequence error" << ecl::endl;
        }
        if (r.response & R1::addr_err) {
            ecl::cout << "Address error" << ecl::endl;
        }
        if (r.response & R1::param_error) {
            ecl::cout << "Parameter error" << ecl::endl;
        }

        rc = err::generic; // TODO: better error code
    }

    return rc;
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::receive_response(R3 &r)
{
    err rc = receive_response(r.r1);
    if (is_error(rc)) {
        return rc;
    }

    return spi_receive((uint8_t *)&r.OCR, sizeof(r.OCR));
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::receive_data(uint8_t *buf, size_t size)
{
    // Data token that returned in case of success
    static constexpr uint8_t data_token    = 0xfe;

    // Flags that can be found in error token
    static constexpr uint8_t generic_err  = 0x01;
    static constexpr uint8_t cc_err        = 0x02;
    static constexpr uint8_t ecc_fail      = 0x04;
    static constexpr uint8_t out_of_range  = 0x08;
    static constexpr uint8_t card_locked   = 0x10;

    uint8_t  token = 0;
    uint16_t crc;
    uint8_t  tries = 64;
    err      rc;

    do {
        rc = spi_receive(&token, sizeof(token));
        if (is_error(rc)) {
            return rc;
        }
    } while (token == 0xff && --tries);

    if (!tries) {
        ecl::cout << "Timeout waiting for data token" << ecl::endl;
        return rc;
    }

    // Data token received,
    if (token == data_token) {
        rc = spi_receive(buf, size);
        if (is_error(rc)) {
            return rc;
        }
    } else {
        ecl::cout << "Error token " << token
                  << " received when reading data:" << ecl::endl;
        if (token & generic_err) {
            ecl::cout << "General error occurs" << ecl::endl;
        }
        if (token & cc_err) {
            ecl::cout << "CC error" << ecl::endl;
        }
        if (token & ecc_fail) {
            ecl::cout << "ECC failed" << ecl::endl;
        }
        if (token & out_of_range) {
            ecl::cout << "Out of range error" << ecl::endl;
        }
        if (token & card_locked) {
            ecl::cout << "Card locked" << ecl::endl;
        }

        return err::generic; // TODO: better error code
    }

    return spi_receive((uint8_t *)&crc, sizeof(crc));
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::send_data(const uint8_t *buf, size_t size)
{
    // Data token that must be sent before data chunk
    static constexpr uint8_t data_token    = 0xfe;

    // Two bits indicating data response
    static constexpr uint8_t mask          = 0x11;
    // Flags that can be found in data response
    static constexpr uint8_t accepted      = 0x05;
    static constexpr uint8_t crc_err       = 0x0b;
    static constexpr uint8_t write_err     = 0x0d;

    static constexpr uint8_t crc           = 0x0;

    uint8_t  data_response = 0;
    uint8_t  tries = 32;
    err      rc;

    // Token
    rc = spi_send(&data_token, sizeof(data_token));
    if (is_error(rc)) {
        return rc;
    }

    // Data itself
    rc = spi_send(buf, size);
    if (is_error(rc)) {
        return rc;
    }

    // Dummy CRC
    rc = spi_send(&crc, sizeof(crc));
    if (is_error(rc)) {
        return rc;
    }

    // Wait for data response
    do {
        rc = spi_receive(&data_response, sizeof(data_response));
        if (is_error(rc)) {
            return rc;
        }
    } while (((data_response & mask) != 0x1) && --tries);

    // No error occur, only 4 lower bits matters
    if ((data_response & 0x0f) == accepted) {
        // Wait till card is busy
        do {
            rc = spi_receive(&data_response, sizeof(data_response));
            if (is_error(rc)) {
                return rc;
            }
        } while (data_response == 0x0);

        return rc;
    }

    ecl::cout << "Error in data response " << data_response << ecl::endl;
    if (data_response & crc_err) {
        ecl::cout << "CRC error occurs" << ecl::endl;
    }
    if (data_response & write_err) {
        ecl::cout << "Write error" << ecl::endl;
    }

    return err::generic; // TODO: better error code
}

//------------------------------------------------------------------------------

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::CMD0(R1 &r)
{
    // TODO: comments
    constexpr uint8_t  CMD0_idx = 0;
    constexpr uint8_t  CMD0_crc = 0x95;
    constexpr argument arg      = {0};
    return send_CMD(r, CMD0_idx, arg, CMD0_crc);
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::CMD8(R7 &r)
{
    // TODO: comments
    constexpr uint8_t   CMD8_idx = 8;
    constexpr uint8_t   CMD8_crc = 0x87;
    constexpr argument  arg      = { 0, 0, 0x01, 0xaa };
    return send_CMD(r, CMD8_idx, arg, CMD8_crc);
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::CMD10(R1 &r)
{
    // TODO: comments
    constexpr uint8_t   CMD10_idx  = 10;
    constexpr argument  arg        = { 0 };
    return send_CMD(r, CMD10_idx, arg);
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::CMD16(R1 &r)
{
    // TODO: comments
    constexpr uint8_t  CMD16_idx = 16;
    constexpr uint32_t block_len = block_buffer::block_len;
    constexpr argument arg = {
        (uint8_t) (block_len >> 24),
        (uint8_t) (block_len >> 16),
        (uint8_t) (block_len >> 8),
        (uint8_t) (block_len),
    };
    return send_CMD(r, CMD16_idx, arg);
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::CMD17(R1 &r, uint32_t address)
{
    // TODO: comments
    constexpr uint8_t CMD17_idx = 17;
    const argument arg = {
        (uint8_t) (address >> 24),
        (uint8_t) (address >> 16),
        (uint8_t) (address >> 8),
        (uint8_t) (address),
    };
    return send_CMD(r, CMD17_idx, arg);
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::CMD24(R1 &r, uint32_t address)
{
    constexpr uint8_t CMD24_idx = 24;
    const argument arg = {
        (uint8_t) (address >> 24),
        (uint8_t) (address >> 16),
        (uint8_t) (address >> 8),
        (uint8_t) (address),
    };
    return send_CMD(r, CMD24_idx, arg);
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::CMD55(R1 &r)
{
    // TODO: comments
    constexpr uint8_t CMD55_idx = 55;
    constexpr argument arg      = { 0, 0, 0, 0 };
    return send_CMD(r, CMD55_idx, arg);
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::CMD58(R3 &r)
{
    // TODO: comments
    constexpr uint8_t CMD58_idx = 58;
    constexpr argument arg      = { 0, 0, 0, 0 };
    return send_CMD(r, CMD58_idx, arg);
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::ACMD41(R1 &r, bool HCS)
{
    const uint8_t HCS_byte = HCS ? (1 << 6) : 0;

    err rc = CMD55(r);
    if (is_error(rc)) {
        return rc;
    }

    // TODO: comments
    constexpr uint8_t  CMD41_idx  = 41;
    const argument     arg        = { HCS_byte, 0, 0, 0 };
    return send_CMD(r, CMD41_idx, arg);
}

//------------------------------------------------------------------------------

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::open_card()
{
    err rc;
    sd_type type;

    rc = software_reset();
    if (is_error(rc)) {
        ecl::cout << "Failed to reset a card" << ecl::endl;
        return rc; // TODO: better error code
    }

    rc = check_conditions();
    if (is_error(rc)) {
        ecl::cout << "Failed to check conditions" << ecl::endl;
        return rc;
    }

    rc = init_process();
    if (is_error(rc)) {
        ecl::cout << "Failed to init a card" << ecl::endl;
        return rc;
    }

    rc = check_OCR(type);
    if (is_error(rc)) {
        ecl::cout << "Failed to check OCR" << ecl::endl;
        return rc;
    } else if (type == sd_type::hc) {
        m_ctx.hc = true;
    } else if (type == sd_type::sc) {
        rc = set_block_length();
        if (is_error(rc)) {
            ecl::cout << "Can't set block length" << ecl::endl;
            return rc;
        }
    } else {
        ecl::cout << "Unsupported type of card" << ecl::endl;
        return err::generic; // TODO: better error code
    }

    ecl::cout << "Card initialized successfully" << ecl::endl;

    rc = obtain_card_info();
    if (is_error(rc)) {
        ecl::cout << "Failed to obtain card info" << ecl::endl;
        return rc;
    }

    rc = populate_block(m_offt);
    if (is_error(rc)) {
        ecl::cout << "Cannot populate initial block" << ecl::endl;
    }

    return rc;
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::software_reset()
{
    R1 r1;

    err rc = CMD0(r1);
    if (is_error(rc)) {
        return rc;
    }

    if (r1.response & R1::idle_state) {
        ecl::cout << "Card is in idle state" << ecl::endl;
        return err::ok;
    }

    return err::generic; // TODO: better error code
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::check_conditions()
{
    R7 r7;

    err rc = CMD8(r7);
    if (is_error(rc)) {
        return rc;
    }

    uint32_t OCR = ecl::BE(r7.OCR);

    if (r7.r1.response & R1::idle_state) {
        if ((OCR & 0xff) != 0xaa) {
            ecl::cout << "Check pattern mismatch" << ecl::endl;
            return err::generic; // TODO: better error code
        }

        if (!(OCR & 0x100)) {
            ecl::cout << "Voltage range not accepted" << ecl::endl;
            return err::geneic; // TODO: better error code
        }

        return err::ok;
    }

    return err::geneic; // TODO: better error code
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::init_process()
{
    //TODO: comments
    R1 r1;
    err rc;

    r1.response = 0x1;
    uint8_t tries = 255;

    while ((r1.response & R1::idle_state) && --tries) {
        rc = ACMD41(r1, true);

        if (is_error(rc)) {
            return rc;
        }
    }

    if (!tries) {
        ecl::cout << "Card wasn't initialized within given period" << ecl::endl;
        return err::generic; // TODO: better error code
    }

    return err::ok;
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::check_OCR(sd_type &type)
{
    R3 r3;
    CMD58(r3);
    err rc;

    uint32_t OCR = ecl::BE(r3.OCR);

    // TODO: compare current profile and supported one!
    // Otherwise we can damage the card
    if (!r3.r1.response) {
        // Card sends data (as well as OCR) in BE format
        if (OCR & (1 << 31)) {
            ecl::cout << "Card is powered-up" << ecl::endl;
            if (OCR & (1 << 30)) {
                ecl::cout << "Card is High capacity" << ecl::endl;
                type = sd_type::hc;
            } else {
                ecl::cout << "Card is Standard capacity" << ecl::endl;
                type = sd_type::sc;
            }
        }

        // Whole byte 2 of OCR contains voltage profiles
        for (uint8_t bit = 15; bit < 24; ++bit) {
            if (OCR & (1 << bit)) {
                uint8_t V = 27 + bit - 15;
                uint intgr = V / 10;
                uint fract = V - intgr * 10;
                ecl::cout << "VOLTAGE: ["
                          << intgr << '.' << fract
                          << "; +0.1]" << ecl::endl;
            }
        }

        return rc;
    }

    return err::generic; // TODO: better error code
}

template<class spi_dev, class gpio_cs>
err sd_spi<spi_dev, gpio_cs>::obtain_card_info()
{
    R1  r1;
    err rc;
    uint8_t cid[16];

    rc = CMD10(r1);
    if (is_error(rc)) {
        return rc;
    }

    rc = receive_data(cid, sizeof(cid));
    if (is_error(rc)) {
        return rc;
    }

    ecl::cout << "------------------------------------" << ecl::endl;
    ecl::cout << "Manufacturer ID:       " << cid[0] << ecl::endl;
    ecl::cout << "OEM/Application ID:    " << (char) cid[1]
              << (char) cid[2] << ecl::endl;

    ecl::cout << "Product name:          ";
    for (uint i = 3; i < 8; i++)
        ecl::cout << (char) cid[i];
    ecl::cout << ecl::endl;
    ecl::cout << "Product revision:      ";
    ecl::cout << (cid[8] >> 4) << '.' << (cid[8] & 0xf) << ecl::endl;
    ecl::cout << "S/N:                   "
              << ((cid[9] << 24) | (cid[10] << 16)
            | (cid[11] << 8) | cid[12]) << ecl::endl;

    uint8_t month = cid[1] & 0xf;
    uint16_t year = 2000 + ((cid[1] >> 4) | (cid[2] & 0xf));

    ecl::cout << "Date:                  " << month << '.' << year << ecl::endl;
    ecl::cout << "------------------------------------" << ecl::endl;
    return err::ok;
}

template<class spi_dev, class gpio_cs>
int sd_spi<spi_dev, gpio_cs>::set_block_length()
{
    R1 r1;

    int SD_ret = CMD16(r1);
    if (SD_ret < 0)
        return SD_ret;

    if (r1.response & R1::idle_state) {
        ecl::cout << "Card is in idle state during CMD16" << ecl::endl;
        SD_ret = sd_err;
    }

    return SD_ret;
}


template<class spi_dev, class gpio_cs>
int sd_spi<spi_dev, gpio_cs>::populate_block(size_t new_block)
{
    R1 r1;
    off_t address = m_HC ? new_block : new_block * block_buffer::block_len;
    int SD_ret = flush_block();

    if ((SD_ret = CMD17(r1, address)) < 0) {
        return SD_ret;
    }

    if ((SD_ret = receive_data(m_block.block, block_buffer::block_len)) < 0) {
        return SD_ret;
    }

    m_block.origin = new_block;
    return SD_ret;
}

template<class spi_dev, class gpio_cs>
int sd_spi<spi_dev, gpio_cs>::flush_block()
{
    int SD_ret;
    R1 r1;

    if (m_block.mint) {
        return sd_ok;
    }

    off_t address = m_HC ? m_block.origin : m_block.origin * block_buffer::block_len;

    if ((SD_ret = CMD24(r1, address)) < 0) {
        return SD_ret;
    }

    if ((SD_ret = send_data(m_block.block, block_buffer::block_len)) < 0) {
        return SD_ret;
    }

    m_block.mint = true;
    return SD_ret;
}

template<class spi_dev, class gpio_cs>
int sd_spi<spi_dev, gpio_cs>::traverse_data(
        size_t count,
        const std::function< void (size_t, size_t, size_t) > &fn
        )
{
    size_t left = count;
    int SD_ret = sd_ok;

    // Intended to be optimized in right shift, sinse
    // block length is constant and a power of two
    size_t blk_num = m_offt / block_buffer::block_len;
    size_t blk_offt = m_offt - blk_num  * block_buffer::block_len;
    size_t data_offt = 0;

    while (left) {
        if (blk_num  != m_block.origin) {
            spi_dev::lock();
            gpio_cs::reset();
            SD_ret = populate_block(blk_num);
            gpio_cs::set();
            spi_dev::unlock();

            if (SD_ret < 0)
                return SD_ret;
        }

        size_t to_copy = std::min(block_buffer::block_len - blk_offt , left);
        // Copy data
        fn(data_offt, blk_offt, to_copy);

        // Advance to next free chunk
        data_offt += to_copy;
        left -= to_copy;

        // If needed, next iteration will populate buffer again
        blk_num ++;
        // Next copy will occur within a new block
        blk_offt  = 0;
    }

    m_offt += count;

    return SD_ret;
}

} // namespace ecl

#endif // DEV_SDSPI_H

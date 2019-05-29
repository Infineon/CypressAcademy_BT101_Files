/*
 * ota_fw_upgrade_common.c
 *
 * WICED Bluetooth OTA Upgrade
 *
 * This file provides functions required for image verification
 * once the download is completed
 */
#include "bt_types.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_firmware_upgrade.h"
#include "wiced_timer.h"
#include "ota_fw_upgrade.h"
#include "sha2.h"
#include "p_256_multprecision.h"
#include "p_256_ecc_pp.h"

extern uint32_t update_crc32( uint32_t crc, uint8_t *buf, uint16_t len );
extern uint32_t update_crc(uint32_t crc, uint8_t *buf, uint16_t len);
extern UINT32 crc32_Update( UINT32 crc, UINT8 *buf, UINT16 len );
extern wiced_result_t  wiced_ota_firmware_upgrade_send_status( uint16_t handle, uint8_t status );
extern wiced_bool_t wiced_ota_indication_enabled( void );

#ifdef OTA_UPGRADE_DEBUG
void dump_hex_cmn(uint8_t *p, uint32_t len);
#endif

Point                                        *p_ecdsa_public_key_cmn = NULL;
ota_fw_upgrade_state_t                       ota_fw_upgrade_common_state;
wiced_ota_firmware_upgrade_status_callback_t *ota_fw_upgrade_status_callback_cmn = NULL;

static  uint8_t      ds_image_prefix[8]         =    {'B', 'R', 'C', 'M', 'c', 'f', 'g', 'D'};

/*
 * verify function is called after all the data has been received and stored
 * in the NV.  The function reads back data from the NV and calculates the checksum.
 * Function returns TRUE if calculated CRC matches the one calculated by the host
 */
int32_t ota_fw_upgrade_verify_cmn( int32_t total_len, uint32_t received_crc )
{
    uint32_t offset;
    uint32_t crc32 = 0xffffffff;

    for (offset = 0; offset < total_len; offset += OTA_FW_UPGRADE_READ_CHUNK)
    {
        uint8_t memory_chunk[OTA_FW_UPGRADE_READ_CHUNK];
        int32_t bytes_to_read = ((offset + OTA_FW_UPGRADE_READ_CHUNK) < total_len) ?
                                        OTA_FW_UPGRADE_READ_CHUNK : total_len - offset;

        // read should be on the word boundary and in full words, we may read a bit more, but
        // include correct number of bytes in the CRC calculation
        wiced_firmware_upgrade_retrieve_from_nv(offset, memory_chunk, (bytes_to_read + 3) & 0xFFFFFFFC);
#if OTA_CHIP == 20703
        crc32 = update_crc(crc32, memory_chunk, bytes_to_read);
#else
#if CHIP_REV_A_20735B1 == 1
        crc32 = crc32_Update(crc32, memory_chunk, bytes_to_read);
#else
        crc32 = update_crc32(crc32, memory_chunk, bytes_to_read);
#endif
#endif

#ifdef OTA_UPGRADE_DEBUG
        //WICED_BT_TRACE("read offset:%x\n", offset);
        //dump_hex_cmn(memory_chunk, bytes_to_read);
#endif
    }
    crc32 = crc32 ^ 0xffffffff;

#ifdef OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("stored crc:%4x  recvd crc:%4x\n", crc32, received_crc );
#endif
    return (crc32 == received_crc );
}

/*
 * verify function is called after all the data has been received and stored
 * in the NV.  The function reads back data from the NV and calculates the checksum.
 * Function returns TRUE if calculated CRC matches the one calculated by the host
 */
int32_t ota_sec_fw_upgrade_verify_cmn( int32_t total_len )
{
    sha2_context sha2_ctx;
    uint32_t     offset;
    uint32_t     nvram_len = total_len - SIGNATURE_LEN;
    uint8_t      hash[32];
    uint8_t      signature[SIGNATURE_LEN + 4];
    uint8_t      res;

    // initialize sha256 context
    sha2_starts(&sha2_ctx, 0);

    for (offset = 0; offset < nvram_len; offset += OTA_SEC_FW_UPGRADE_READ_CHUNK)
    {
        uint8_t memory_chunk[OTA_SEC_FW_UPGRADE_READ_CHUNK];
        int32_t bytes_to_read = ((offset + OTA_SEC_FW_UPGRADE_READ_CHUNK) < nvram_len) ? OTA_SEC_FW_UPGRADE_READ_CHUNK : nvram_len - offset;

        // read should be on in full words, we may read a bit more, but include correct number of bytes in the hash calculation
        if (wiced_firmware_upgrade_retrieve_from_nv(offset, memory_chunk, (bytes_to_read + 3) & 0xFFFFFFFC) != ((bytes_to_read + 3) & 0xFFFFFFFC))
        {
            WICED_BT_TRACE("failed to read loc0:%x\n", offset);
        }

        // dump_hex_cmn(memory_chunk, bytes_to_read);
        sha2_update(&sha2_ctx, memory_chunk, bytes_to_read);
    }
    sha2_finish(&sha2_ctx, hash);

#ifdef OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("hash:\n");
    dump_hex_cmn(hash, sizeof(hash));

    WICED_BT_TRACE("public_key:%x\n", (uint8_t *)p_ecdsa_public_key_cmn);
    dump_hex_cmn((uint8_t *)p_ecdsa_public_key_cmn, sizeof(Point));
#endif
    // read should be on the word boundary and in full words. Need to adjust offset to full words and read a bit more.
    offset = total_len - SIGNATURE_LEN;
    if (wiced_firmware_upgrade_retrieve_from_nv(offset -  (offset & 0x03), signature, SIGNATURE_LEN + 4) != SIGNATURE_LEN + 4)
    {
        WICED_BT_TRACE("failed to read loc1:%x\n", offset -  (offset & 0x03));
    }

#ifdef OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("signature:\n");
    dump_hex_cmn(signature + (offset & 0x03), SIGNATURE_LEN);
#endif
    res = ecdsa_verify_(hash, signature + (offset & 0x03), p_ecdsa_public_key_cmn);
#ifdef OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("ecdsa_verify_:%d", res);
#endif
    return res;
}

/*
 * This function can be used if the server/client wants to calculate the checksum
 * of the OTA upgrade data
 */
int32_t ota_fw_upgrade_calculate_checksum( int32_t offset, int32_t length )
{

    uint32_t read_start = 0;
    uint32_t crc32 = 0xffffffff;
    uint32_t total_len = offset + length;

    for (; offset < total_len; offset += OTA_FW_UPGRADE_READ_CHUNK)
    {
        uint8_t memory_chunk[OTA_FW_UPGRADE_READ_CHUNK];
        int32_t bytes_to_read;
        uint8_t mod_val = ( offset % OTA_FW_UPGRADE_READ_CHUNK );

        //To ensure aligned read
        if ( ( !read_start ) && ( mod_val ) )
        {
            bytes_to_read = ( ( OTA_FW_UPGRADE_READ_CHUNK ) < total_len ) ?
                    ( OTA_FW_UPGRADE_READ_CHUNK - mod_val ) : total_len - mod_val;
        }
        else
        {
            bytes_to_read = ((offset + OTA_FW_UPGRADE_READ_CHUNK) < total_len) ?
                                        OTA_FW_UPGRADE_READ_CHUNK : total_len - offset;
        }
        read_start = 1;

        // read should be on the word boundary and in full words, we may read a bit more, but
        // include correct number of bytes in the CRC calculation
        wiced_firmware_upgrade_retrieve_from_nv(offset, memory_chunk, (bytes_to_read + 3) & 0xFFFFFFFC);
#if OTA_CHIP == 20703
        crc32 = update_crc(crc32, memory_chunk, bytes_to_read);
#else
#if CHIP_REV_A_20735B1 == 1
        crc32 = crc32_Update(crc32, memory_chunk, bytes_to_read);
#else
        crc32 = update_crc32(crc32, memory_chunk, bytes_to_read);
#endif
#endif

#ifdef OTA_UPGRADE_DEBUG
        //WICED_BT_TRACE("read offset:%x\n", offset);
        //dump_hex_cmn(memory_chunk, bytes_to_read);
#endif
    }
    crc32 = crc32 ^ 0xffffffff;

    return crc32;
}
/*
 * Process timeout started after the last notification to perform restart
 */
void ota_fw_upgrade_common_reset_timeout(uint32_t param)
{
    wiced_firmware_upgrade_finish();
}

/*
 * handle OTA upgrade commands
 */
wiced_bool_t ota_fw_upgrade_command_handler(uint16_t conn_id, uint8_t command, uint8_t *data, int32_t len)
{
    ota_fw_upgrade_state_t *p_state = &ota_fw_upgrade_common_state;
    uint8_t value = WICED_OTA_UPGRADE_STATUS_OK;
    int32_t verified = WICED_FALSE;

#ifdef OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("OTA handle cmd:%d, state:%d\n", command, p_state->state);
#endif
    if (command == WICED_OTA_UPGRADE_COMMAND_PREPARE_DOWNLOAD)
    {
        p_state->state = OTA_STATE_READY_FOR_DOWNLOAD;
        wiced_ota_firmware_upgrade_send_status(conn_id, value);

        if (ota_fw_upgrade_status_callback_cmn)
        {
            (*ota_fw_upgrade_status_callback_cmn)(OTA_FW_UPGRADE_STATUS_STARTED);
        }
        return WICED_TRUE;
    }
    if (command == WICED_OTA_UPGRADE_COMMAND_ABORT)
    {
        p_state->state = OTA_STATE_ABORTED;
        wiced_ota_firmware_upgrade_send_status(conn_id, value);

        if (ota_fw_upgrade_status_callback_cmn)
        {
            (*ota_fw_upgrade_status_callback_cmn)(OTA_FW_UPGRADE_STATUS_ABORTED);
        }
        return WICED_FALSE;
    }

    switch (p_state->state)
    {
    case OTA_STATE_IDLE:
        return WICED_TRUE;

    case OTA_STATE_READY_FOR_DOWNLOAD:
        if (command == WICED_OTA_UPGRADE_COMMAND_DOWNLOAD)
        {
            // command to start upgrade should be accompanied by 4 bytes with the image size
            if (len < 4)
            {
                WICED_BT_TRACE("Bad Download len: %d \n", len);
                return WICED_FALSE;
            }

            if (!wiced_firmware_upgrade_init_nv_locations())
            {
                WICED_BT_TRACE("failed init nv locations\n");
                value = WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE;
                wiced_ota_firmware_upgrade_send_status(conn_id, value);
                return WICED_FALSE;
            }

            p_state->state                = OTA_STATE_DATA_TRANSFER;
            p_state->current_offset       = 0;
            p_state->current_block_offset = 0;
            p_state->total_offset         = 0;
            p_state->total_len            = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
#if OTA_UPGRADE_DEBUG
            p_state->recv_crc32           = 0xffffffff;
#endif

#if ( defined(CYW20719B0) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW20735B1) /*|| defined (CYW20819A1)*/)
            // if we are using Secure version the total length comes in the beginning of the image,
            // do not use the one from the downloader.
            if (p_ecdsa_public_key_cmn != NULL)
            {
                p_state->total_len            = 0;
            }
            else
            {
                p_state->total_len            = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
            }
#endif
            WICED_BT_TRACE("state %d total_len %d \n", p_state->state, data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
            wiced_ota_firmware_upgrade_send_status(conn_id, value);
            return WICED_TRUE;
        }
        break;

    case OTA_STATE_DATA_TRANSFER:
        if (command == WICED_OTA_UPGRADE_COMMAND_VERIFY)
        {
            // command to start upgrade should be accompanied by 2 bytes with the image size
            if (len < 4)
            {
                WICED_BT_TRACE("Bad Verify len %d \n", len);
                return WICED_FALSE;
            }
            // command to perform verification.
            if (p_state->total_len != p_state->total_offset)
            {
                WICED_BT_TRACE("Verify failed received:%d out of %d\n", p_state->total_offset, p_state->total_len);
                p_state->state = OTA_STATE_ABORTED;
                value = WICED_OTA_UPGRADE_STATUS_VERIFICATION_FAILED;
                wiced_ota_firmware_upgrade_send_status(conn_id, value);

                if (ota_fw_upgrade_status_callback_cmn)
                {
                    (*ota_fw_upgrade_status_callback_cmn)(OTA_FW_UPGRADE_STATUS_ABORTED);
                }
                return WICED_FALSE;
            }

            // For none-secure case the command should have 4 bytes CRC32
            if (p_ecdsa_public_key_cmn == NULL)
            {
                p_state->crc32 = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
                verified = ota_fw_upgrade_verify_cmn(p_state->total_len,p_state->crc32);
            }
            else
            {
                WICED_BT_TRACE("ota_sec_fw_upgrade_verify_cmn() \n");
                verified = ota_sec_fw_upgrade_verify_cmn(p_state->total_len);
            }

            if (!verified)
            {
                WICED_BT_TRACE("Verify failed\n");
                p_state->state = OTA_STATE_ABORTED;
                value = WICED_OTA_UPGRADE_STATUS_VERIFICATION_FAILED;
                wiced_ota_firmware_upgrade_send_status(conn_id, value);

                if (ota_fw_upgrade_status_callback_cmn)
                {
                    (*ota_fw_upgrade_status_callback_cmn)(OTA_FW_UPGRADE_STATUS_ABORTED);
                }
                return WICED_FALSE;
            }
            WICED_BT_TRACE("Verify success\n");
            p_state->state = OTA_STATE_VERIFIED;

            if ( WICED_SUCCESS == wiced_ota_firmware_upgrade_send_status(conn_id, value) )
            {
                if ( wiced_ota_indication_enabled() )
                {
                    return WICED_TRUE;
                }
                else
                {
                    //notify application that we are going down
                    if (ota_fw_upgrade_status_callback_cmn)
                    {
                        (*ota_fw_upgrade_status_callback_cmn)(OTA_FW_UPGRADE_STATUS_COMPLETED);
                    }
                    // init timer for detect packet retransmission
                    wiced_deinit_timer(&ota_fw_upgrade_common_state.reset_timer);
                    wiced_init_timer(&ota_fw_upgrade_common_state.reset_timer, ota_fw_upgrade_common_reset_timeout, 0, WICED_SECONDS_TIMER);
                    wiced_start_timer(&ota_fw_upgrade_common_state.reset_timer, 1);
                    return WICED_TRUE;
                }
            }

            WICED_BT_TRACE("failed to notify the peer\n");
            return WICED_FALSE;
        }
        break;

    case OTA_STATE_ABORTED:
    default:
        break;
    }

    value = WICED_OTA_UPGRADE_STATUS_ILLEGAL_STATE;
    wiced_ota_firmware_upgrade_send_status(conn_id, value);
    return WICED_FALSE;
}

/*
 * Process data chunk received from the host.  If received num of bytes equals to
 * OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT, save the data to NV.
 *
 */
wiced_bool_t ota_fw_upgrade_image_data_handler(uint16_t conn_id, uint8_t *data, int32_t len)
{
    ota_fw_upgrade_state_t *p_state = &ota_fw_upgrade_common_state;
    uint8_t *p = data;

    if (p_state->state != OTA_STATE_DATA_TRANSFER)
        return FALSE;

// Image prefixes are supported on 20719xx and 20735
#ifndef CYW20706A2
    // For the Secure upgrade, verify the Product info
    if (p_ecdsa_public_key_cmn != NULL)
    {
        // If this is the first chunk of the image, we need to verify the header and extract the length
        // Following check is for the FW2
        if (p_state->total_len == 0)
        {
            if (memcmp(data, ds_image_prefix, sizeof(ds_image_prefix)) != 0)
            {
                WICED_BT_TRACE("Bad data start\n");
                return (FALSE);
            }
            // length store in the image does not include size of ds_image_prefix
            p_state->total_len = data[12] + (data[13] << 8) + (data[14] << 16) + (data[15] << 24);
            p_state->total_len += DS_IMAGE_PREFIX_LEN + SIGNATURE_LEN;

            // ToDo validate flash size
            // ToDo validate that product is the same as stored and major is >= the one that stored
            WICED_BT_TRACE("Image for Product 0x%x Major:%d Minor:%d len:%d\n", data[8] + (data[9] << 8), data[10], data[11], p_state->total_len);
        }
    }
    else
#endif
    {
#if OTA_UPGRADE_DEBUG
        // For testing calculate received CRC32 of the received data
#ifdef CYW20706A2
        p_state->recv_crc32 = update_crc(p_state->recv_crc32, data, len);
#else
#ifdef WICEDX
        p_state->recv_crc32 = update_crc32(p_state->recv_crc32, data, len);
#elif CYW20735B1
        p_state->recv_crc32 = crc32_Update(p_state->recv_crc32, data, len);
#else
        p_state->recv_crc32 = update_crc32(p_state->recv_crc32, data, len);
#endif
#endif
#endif
    }

    while (len)
    {
        int bytes_to_copy =
            (p_state->current_block_offset + len) < OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT ? len: (OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT - p_state->current_block_offset);

        if ((p_state->total_offset + p_state->current_block_offset + bytes_to_copy > p_state->total_len))
        {
            WICED_BT_TRACE("Too much data. size of the image %d offset %d, block offset %d len rcvd %d\n",
                    p_state->total_len, p_state->total_offset, p_state->current_block_offset, len);
            return (FALSE);
        }

        memcpy (&(p_state->read_buffer[p_state->current_block_offset]), p, bytes_to_copy);
        p_state->current_block_offset += bytes_to_copy;

        if ((p_state->current_block_offset == OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT) ||
            (p_state->total_offset + p_state->current_block_offset == p_state->total_len))
        {
#if OTA_UPGRADE_DEBUG
            WICED_BT_TRACE("write offset:%x\n", p_state->total_offset);
            //dump_hex_cmn(p_state->read_buffer, p_state->current_block_offset);
#endif
            // write should be on the word boundary and in full words, we may write a bit more
            const int current_block_len = (p_state->current_block_offset + 3) & 0xfffffffc;
            if (current_block_len != wiced_firmware_upgrade_store_to_nv(
                        p_state->total_offset, p_state->read_buffer, current_block_len))
            {
                return FALSE;
            }
            p_state->total_offset        += p_state->current_block_offset;
            p_state->current_block_offset = 0;

#if OTA_UPGRADE_DEBUG
            if (p_state->total_offset == p_state->total_len)
            {
                p_state->recv_crc32 = p_state->recv_crc32 ^ 0xffffffff;
                WICED_BT_TRACE("recv_crc32:%x\n", p_state->recv_crc32);
            }
#endif
        }

        len = len - bytes_to_copy;
        p = p + bytes_to_copy;

        //WICED_BT_TRACE("remaining len: %d \n", len);
    }
    return (TRUE);
}

#ifdef OTA_UPGRADE_DEBUG
void dump_hex_cmn(uint8_t *p, uint32_t len)
{
    uint32_t i;
    char     buff1[100];

    while (len != 0)
    {
        memset(buff1, 0, sizeof(buff1));
        for (i = 0; i < len && i < 32; i++)
        {
            int s1 = (*p & 0xf0) >> 4;
            int s2 = *p & 0x0f;
            buff1[i * 3]     = (s1 >= 0 && s1 <= 9) ? s1 + '0' : s1 - 10 + 'A';
            buff1[i * 3 + 1] = (s2 >= 0 && s2 <= 9) ? s2 + '0' : s2 - 10 + 'A';
            buff1[i * 3 + 2] = ' ';
            p++;
        }
        len -= i;
        if (len != 0)
            WICED_BT_TRACE("%s\n", buff1);
    }
    WICED_BT_TRACE("%s\n", buff1);
}
#endif

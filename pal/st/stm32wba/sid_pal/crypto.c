/**
  ******************************************************************************
  * @file    crypto.c
  * @brief   sid_pal_crypto module implementation for STM32WBAxx MCUs
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* Sidewalk interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_crypto_ifc.h>
#include <sid_pal_log_ifc.h>

/* Platform-specific headers */
#include "app_conf.h"
#include <cmsis_os.h>
#include <crc_ctrl.h>
#include <hw.h>
#include <stm32wbaxx_hal.h>

/* Crypto library */
#include "cmox_crypto.h"

/* Utilities and helpers */
#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#define AES_KEY_SZ (16u)
#define BUFF_SIZE  (2200u) /*!< Size of the working buffer for the crypto library. This amount of RAM is reserved exclusively for the cryptography needs */

#define SID_PAL_CRYPTO_CRC_HWADDR (CRC) /*!< Physical address of the CRC to use */

/* Private variables ---------------------------------------------------------*/

static bool hal_init_done;

/* ECC context */
static cmox_ecc_handle_t ecc_ctx;
/* ECC working buffer */
static uint8_t working_buffer[BUFF_SIZE];
/* Temporary storage for CRC peripheral register to allow shared usage of CRC */
static CRC_TypeDef hcrc_context_storage;

/* Private constants ---------------------------------------------------------*/

/* x25519 curve basepoint as per specification */
static const uint8_t x25519_base_point[CMOX_ECC_CURVE25519_PUBKEY_LEN] = {
    0x09u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
};

/* Private function prototypes -----------------------------------------------*/

static sid_error_t stm32wba_crypto_init(void);
static sid_error_t stm32wba_crypto_deinit(void);
static inline void stm32wba_crypto_obtain_hw_control(void);
static inline void stm32wba_crypto_release_hw_control(void);
static sid_error_t stm32wba_crypto_rand(uint8_t * const rand, const size_t size);
static cmox_ecc_retval_t stm32wba_crypto_hash(sid_pal_hash_params_t * const params, const bool needs_hw_lock);
static sid_error_t stm32wba_crypto_hmac(sid_pal_hmac_params_t * const params);
static sid_error_t stm32wba_crypto_aes_crypt(sid_pal_aes_params_t * const params);
static sid_error_t stm32wba_crypto_aead_crypt(sid_pal_aead_params_t *params);
static cmox_hash_retval_t get_hash_ecc_dsa_input(const sid_pal_dsa_params_t * const in_params,
                                                 sid_pal_dsa_params_t * const out_params,
                                                 uint8_t * const hash,
                                                 const size_t hash_size);
static cmox_ecc_retval_t do_ecc_dsa_verify(const sid_pal_dsa_params_t * const params);
static cmox_ecc_retval_t do_ecc_dsa_sign(sid_pal_dsa_params_t * const params);
static cmox_ecc_retval_t do_x25519_keyGen(cmox_ecc_handle_t * const P_pEccCtx,
                                          const uint8_t * const     P_pRandom,
                                          const size_t              P_RandomLen,
                                          uint8_t * const           P_pPrivKey,
                                          size_t * const            P_pPrivKeyLen,
                                          uint8_t * const           P_pPubKey,
                                          size_t * const            P_pPubKeyLen);
static cmox_ecc_retval_t do_ecc_key_gen(sid_pal_ecc_key_gen_params_t * const params);
static sid_error_t stm32wba_crypto_ecc_dsa(sid_pal_dsa_params_t * const params);
static sid_error_t stm32wba_crypto_ecc_ecdh(sid_pal_ecdh_params_t * const params);
static sid_error_t stm32wba_crypto_ecc_key_gen(sid_pal_ecc_key_gen_params_t * const params);

/* Private function definitions ----------------------------------------------*/

static sid_error_t stm32wba_crypto_init(void)
{
    cmox_init_retval_t cmox_ret;

    /* Initialize cryptographic library */
    cmox_ret = cmox_initialize(NULL);
    if (cmox_ret != CMOX_INIT_SUCCESS)
    {
        return SID_ERROR_GENERIC;
    }

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

static sid_error_t stm32wba_crypto_deinit(void)
{
    cmox_init_retval_t cmox_ret;

    cmox_ret = cmox_finalize(NULL);
    if (cmox_ret != CMOX_INIT_SUCCESS)
    {
    	return SID_ERROR_GENERIC;
    }

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void stm32wba_crypto_obtain_hw_control(void)
{
    /* Lock CRC to avoid concurrent access */
    (void)CRCCTRL_MutexTake();

    /* Store original registers of the CRC unit */
    hcrc_context_storage.DR   = SID_PAL_CRYPTO_CRC_HWADDR->DR;
    hcrc_context_storage.IDR  = SID_PAL_CRYPTO_CRC_HWADDR->IDR;
    hcrc_context_storage.INIT = SID_PAL_CRYPTO_CRC_HWADDR->INIT;
    hcrc_context_storage.CR   = SID_PAL_CRYPTO_CRC_HWADDR->CR;
    hcrc_context_storage.POL  = SID_PAL_CRYPTO_CRC_HWADDR->POL;
    __COMPILER_BARRIER();

    /* Configure CRC for sid_pal_crypto */
    SID_PAL_CRYPTO_CRC_HWADDR->INIT = DEFAULT_CRC_INITVALUE;
    SID_PAL_CRYPTO_CRC_HWADDR->POL  = DEFAULT_CRC32_POLY;
    SID_PAL_CRYPTO_CRC_HWADDR->IDR  = 0u;
    SID_PAL_CRYPTO_CRC_HWADDR->CR   = CRC_POLYLENGTH_32B | CRC_CR_RESET;
    __COMPILER_BARRIER();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void stm32wba_crypto_release_hw_control(void)
{
    /* Restore Data Register */
    SID_PAL_CRYPTO_CRC_HWADDR->INIT = hcrc_context_storage.DR;
    SID_PAL_CRYPTO_CRC_HWADDR->CR   = CRC_CR_RESET;
    __COMPILER_BARRIER();

    /* Restore the rest of the registers */
    SID_PAL_CRYPTO_CRC_HWADDR->IDR  = hcrc_context_storage.IDR;
    SID_PAL_CRYPTO_CRC_HWADDR->INIT = hcrc_context_storage.INIT;
    SID_PAL_CRYPTO_CRC_HWADDR->POL  = hcrc_context_storage.POL;
    SID_PAL_CRYPTO_CRC_HWADDR->CR   = hcrc_context_storage.CR;
    __COMPILER_BARRIER();

    /* Release CRC access lock */
    (void)CRCCTRL_MutexRelease();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t stm32wba_crypto_rand(uint8_t * const rand, const size_t size)
{
    size_t    remaining_bytes = size;
    uint8_t * current_pos = rand;

    while (remaining_bytes > 0u)
    {
        /* Wait for HW_RNG to populate the pool of random data */
        HW_RNG_WaitPoolIsFull();

        if (remaining_bytes >= sizeof(uint32_t))
        {
            /* HW_RNG cannot provide more than CFG_HW_RNG_POOL_SIZE words at a time, get as many words as we can */
            uint8_t words_to_process = (remaining_bytes / sizeof(uint32_t)) % (CFG_HW_RNG_POOL_SIZE + 1u);

            HW_RNG_Get(words_to_process, (uint32_t *)(void *)current_pos);

            /* Update counters and pointers */
            current_pos     += (words_to_process * sizeof(uint32_t));
            remaining_bytes -= (words_to_process * sizeof(uint32_t));
        }
        else
        {
            /* Less than 4 bytes remaining, generate one random word and copy the necessary amount of bytes */
            uint32_t random32bit;

            HW_RNG_Get(1u, &random32bit);
            SID_STM32_UTIL_fast_memcpy(current_pos, &random32bit, remaining_bytes);

            /* Done */
            remaining_bytes = 0u;
        }
    }

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static cmox_ecc_retval_t stm32wba_crypto_hash(sid_pal_hash_params_t * const params, const bool needs_hw_lock)
{
    cmox_hash_algo_t cmox_hash_algo;
    cmox_ecc_retval_t retval;
    size_t expected_digest_size;

    switch(params->algo)
    {
        case SID_PAL_HASH_SHA256:
            cmox_hash_algo = CMOX_SHA256_ALGO;
            expected_digest_size = CMOX_SHA256_SIZE;
            break;

    case SID_PAL_HASH_SHA512:
        cmox_hash_algo = CMOX_SHA512_ALGO;
        expected_digest_size = CMOX_SHA512_SIZE;
        break;

    default:
        return CMOX_HASH_ERR_BAD_PARAMETER;
    }

    SID_PAL_ASSERT(expected_digest_size == params->digest_size);
    if (needs_hw_lock != false)
    {
        stm32wba_crypto_obtain_hw_control();
    }
    retval = cmox_hash_compute(cmox_hash_algo,                  /* Use the selected algorithm */
                               params->data, params->data_size, /* Message to digest */
                               params->digest,                  /* Data buffer to receive digest data */
                               expected_digest_size,            /* Expected digest size */
                               &(params->digest_size));         /* Size of computed digest */
    if (needs_hw_lock != false)
    {
        stm32wba_crypto_release_hw_control();
    }

    return retval;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t stm32wba_crypto_hmac(sid_pal_hmac_params_t * const params)
{
    cmox_mac_retval_t retval;
    cmox_mac_algo_t algo;
    size_t expected_len;

    switch(params->algo)
    {
        case SID_PAL_HASH_SHA256:
            algo = CMOX_HMAC_SHA256_ALGO;
            expected_len = CMOX_SHA256_SIZE;
            break;

        case SID_PAL_HASH_SHA512:
            algo = CMOX_HMAC_SHA512_ALGO;
            expected_len = CMOX_SHA512_SIZE;
            break;
        default:
            return SID_ERROR_NOSUPPORT;
    }

    stm32wba_crypto_obtain_hw_control();
    /* Compute directly the authentication tag passing all the needed parameters */
    retval = cmox_mac_compute(algo,                            /* Use selected HMAC algorithm */
                              params->data, params->data_size, /* Message to authenticate */
                              params->key, params->key_size,   /* HMAC Key to use */
                              NULL, 0,                         /* Custom data */
                              params->digest,                  /* Data buffer to receive generated authentication tag */
                              expected_len,                    /* Expected authentication tag size */
                              &(params->digest_size));         /* Generated tag size */
    stm32wba_crypto_release_hw_control();

    if (retval != CMOX_MAC_SUCCESS)
    {
        return SID_ERROR_GENERIC;
    }

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t stm32wba_crypto_aes_crypt(sid_pal_aes_params_t * const params)
{
    cmox_cipher_algo_t algo = NULL;
    cmox_mac_algo_t cmac_algo = NULL;
    size_t key_size_bits = 0u;

    switch (params->algo)
    {
        case SID_PAL_AES_CMAC_128:
             cmac_algo = CMOX_CMAC_AES_ALGO;
            key_size_bits = 128u;
            break;

        case SID_PAL_AES_CTR_128:
            algo = SID_PAL_CRYPTO_DECRYPT == params->mode ? CMOX_AES_CTR_DEC_ALGO : CMOX_AES_CTR_ENC_ALGO;
            key_size_bits = 128u;
            break;

        default:
            return SID_ERROR_NOSUPPORT;
    }

    if ((params->algo == SID_PAL_AES_CTR_128) && (params->iv_size != AES_KEY_SZ))
    {
        return SID_ERROR_INVALID_ARGS;
    }

    if (key_size_bits != params->key_size)
    {
        return SID_ERROR_INVALID_ARGS;
    }

    switch (params->mode)
    {
        case SID_PAL_CRYPTO_ENCRYPT:
            {
                cmox_cipher_retval_t retval;
                stm32wba_crypto_obtain_hw_control();
                retval = cmox_cipher_encrypt(algo,                                 /* Use AES CBC algorithm */
                                             params->in, params->in_size,          /* Plaintext to encrypt */
                                             params->key, (params->key_size / 8u), /* AES key to use */
                                             params->iv, params->iv_size,          /* Initialization vector */
                                             params->out, &(params->out_size));    /* Data buffer to receive generated ciphertext */
                stm32wba_crypto_release_hw_control();

                if (retval != CMOX_CIPHER_SUCCESS)
                {
                    SID_PAL_LOG_ERROR("aes_encrypt[%d %d]. CMOX err: 0x%08x", params->algo, params->mode, retval);
                    return SID_ERROR_GENERIC;
                }
            }
            break;

        case SID_PAL_CRYPTO_DECRYPT:
            {
                cmox_cipher_retval_t retval;
                stm32wba_crypto_obtain_hw_control();
                retval = cmox_cipher_decrypt(algo,                   /* Use AES CBC algorithm */
                               params->in, params->in_size,          /* Ciphertext to decrypt */
                               params->key, (params->key_size / 8u), /* AES key to use */
                               params->iv, params->iv_size,          /* Initialization vector */
                               params->out, &(params->out_size));    /* Data buffer to receive generated plaintext */
                stm32wba_crypto_release_hw_control();

                if (retval != CMOX_CIPHER_SUCCESS)
                {
                    SID_PAL_LOG_ERROR("aes_decrypt[%d %d]. CMOX err: 0x%08x", params->algo, params->mode, retval);
                    return SID_ERROR_GENERIC;
                }
            }
            break;

        case SID_PAL_CRYPTO_MAC_CALCULATE:
            {
                cmox_mac_retval_t retval;
                stm32wba_crypto_obtain_hw_control();
                retval = cmox_mac_compute(cmac_algo,              /* Use AES CMAC algorithm */
                            params->in, params->in_size,          /* Message to authenticate */
                            params->key, (params->key_size / 8u), /* AES key to use */
                            NULL, 0u,          /* Custom data */
                            params->out,                          /* Data buffer to receive generated authnetication tag */
                            AES_KEY_SZ,                           /* Expected authentication tag size */
                            &(params->out_size));                 /* Generated tag size */
                stm32wba_crypto_release_hw_control();

                if (retval != CMOX_MAC_SUCCESS)
                {
                    SID_PAL_LOG_ERROR("aes_mac_compute[%d %d]. CMOX err: 0x%08x", params->algo, params->mode, retval);
                    return SID_ERROR_GENERIC;
                }
            }
            break;

        default:
            return SID_ERROR_INVALID_ARGS;
    }

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t stm32wba_crypto_aead_crypt(sid_pal_aead_params_t *params)
{
    sid_error_t status = SID_ERROR_GENERIC;
    cmox_cipher_retval_t retval;
    cmox_aead_algo_t algo;
    size_t key_size_bits = 0;

    switch (params->algo)
    {
        case SID_PAL_AEAD_GCM_128:
            algo = SID_PAL_CRYPTO_ENCRYPT == params->mode ? CMOX_AES_GCM_ENC_ALGO : CMOX_AES_GCM_DEC_ALGO;
            key_size_bits = 128u;
            break;

        case SID_PAL_AEAD_CCM_128:
        case SID_PAL_AEAD_CCM_STAR_128:
            algo = SID_PAL_CRYPTO_ENCRYPT == params->mode ? CMOX_AES_CCM_ENC_ALGO : CMOX_AES_CCM_DEC_ALGO;
            key_size_bits = 128u;
            break;

        default:
            return SID_ERROR_NOSUPPORT;
    }

    if (key_size_bits != params->key_size)
    {
        return SID_ERROR_INVALID_ARGS;
    }

    switch (params->mode)
    {
        case SID_PAL_CRYPTO_ENCRYPT:
            {
                const size_t expected_mac_size = params->mac_size;
                const size_t expected_out_size = params->in_size;
                size_t computed_size;

                if ((expected_out_size + expected_mac_size) > sizeof(working_buffer))
                {
                    SID_PAL_LOG_ERROR("AEAD encryption: cannot process data, working buffer is too small. Data size: %u", expected_out_size + expected_mac_size);
                    return SID_ERROR_BUFFER_OVERFLOW;
                }

                stm32wba_crypto_obtain_hw_control();
                retval = cmox_aead_encrypt(algo,                                 /* Use selected AES algorithm */
                                           params->in, params->in_size,          /* Plaintext to encrypt */
                                           expected_mac_size,                    /* Authentication tag size */
                                           params->key, (params->key_size / 8u), /* AES key to use */
                                           params->iv, params->iv_size,          /* Initialization vector */
                                           params->aad, params->aad_size,        /* Additional authenticated data */
                                           working_buffer, &(computed_size));    /* Data buffer to receive generated ciphertext
                                                                                    and authentication tag */
                stm32wba_crypto_release_hw_control();

                /* Verify API returned value */
                if (retval != CMOX_CIPHER_SUCCESS)
                {
                    return SID_ERROR_ENCRYPTION_FAIL;
                }

                /* Verify generated data size is the expected one */
                if (computed_size != (expected_out_size + expected_mac_size))
                {
                    return SID_ERROR_ENCRYPTION_FAIL;
                }

                memcpy(params->out, working_buffer, expected_out_size);
                params->out_size = expected_out_size;

                if (params->mac != NULL)
                {
                    memcpy(params->mac, &(working_buffer[expected_out_size]), expected_mac_size);
                    params->mac_size = expected_mac_size;
                }

                status = SID_ERROR_NONE;
            }
            break;

        case SID_PAL_CRYPTO_DECRYPT:
            {
                const size_t total_in_size = params->in_size + params->mac_size;
                const size_t expected_out_size = params->in_size;

                if (expected_out_size > sizeof(working_buffer))
                {
                    SID_PAL_LOG_ERROR("AEAD decryption: cannot process data, working buffer is too small. Data size: %u", expected_out_size);
                    return SID_ERROR_BUFFER_OVERFLOW;
                }

                memcpy(working_buffer, params->in, params->in_size);
                memcpy(&(working_buffer[params->in_size]), params->mac, params->mac_size);

                stm32wba_crypto_obtain_hw_control();
                retval = cmox_aead_decrypt(algo,                                 /* Use selected AES algorithm */
                                           working_buffer, total_in_size,        /* Ciphertext + tag to decrypt and verify */
                                           params->mac_size,                     /* Authentication tag size */
                                           params->key, (params->key_size / 8u), /* AES key to use */
                                           params->iv, params->iv_size,          /* Initialization vector */
                                           params->aad, params->aad_size,        /* Additional authenticated data */
                                           params->out, &(params->out_size));    /* Data buffer to receive generated plaintext */
                stm32wba_crypto_release_hw_control();

                /* Verify API returned value */
                if (retval != CMOX_CIPHER_AUTH_SUCCESS)
                {
                    return SID_ERROR_ENCRYPTION_FAIL;
                }

                /* Verify generated data size is the expected one */
                if (params->out_size != expected_out_size)
                {
                    return SID_ERROR_ENCRYPTION_FAIL;
                }

                status = SID_ERROR_NONE;
            }
            break;

        default:
            status = SID_ERROR_INVALID_ARGS;
            break;
    }

    return status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static cmox_hash_retval_t get_hash_ecc_dsa_input(const sid_pal_dsa_params_t * const in_params,
                                                                          sid_pal_dsa_params_t * const out_params,
                                                                          uint8_t * const hash,
                                                                          const size_t hash_size)
{
    cmox_hash_retval_t ret = CMOX_HASH_ERR_INTERNAL;

    if (in_params->algo == SID_PAL_ECDSA_SECP256R1)
    {
        sid_pal_hash_params_t hash_params = {
            .data           = in_params->in,
            .data_size      = in_params->in_size,
            .digest         = hash,
            .digest_size    = hash_size,
            .algo           = SID_PAL_HASH_SHA256,
        };
        ret =  stm32wba_crypto_hash(&hash_params, false);
        if (ret != CMOX_HASH_SUCCESS)
        {
            return ret;
        }
        *out_params  = *in_params;
        out_params->in = hash;
        out_params->in_size = hash_size;
    }
    else
    {
        *out_params  = *in_params;
        ret = CMOX_HASH_SUCCESS;
    }

    return ret;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static cmox_ecc_retval_t do_ecc_dsa_verify(const sid_pal_dsa_params_t * const params)
{
    cmox_ecc_retval_t retval = CMOX_ECC_ERR_INTERNAL;
    cmox_hash_retval_t hretval;
    uint8_t hash_digest[CMOX_SHA256_SIZE];
    sid_pal_dsa_params_t compat_params;

    switch (params->algo)
    {
        case SID_PAL_EDDSA_ED25519:
            cmox_ecc_construct(&ecc_ctx, CMOX_ECC256_MATH_FUNCS, working_buffer, sizeof(working_buffer));
            retval = cmox_eddsa_verify(&ecc_ctx,                            /* ECC context */
                                       CMOX_ECC_CURVE_ED25519,              /* ED25519 ECC curve selected */
                                       params->key, params->key_size,       /* Public key for verification */
                                       params->in, params->in_size,         /* Message to verify */
                                       params->signature, params->sig_size, /* Data buffer to receive signature */
                                       NULL);
            cmox_ecc_cleanup(&ecc_ctx);
            break;

        case SID_PAL_ECDSA_SECP256R1:
            hretval = get_hash_ecc_dsa_input(params, &compat_params, hash_digest, sizeof(hash_digest));
            if (hretval != CMOX_HASH_SUCCESS)
            {
                SID_PAL_LOG_ERROR("Hash calculation. CMOX err: 0x%08x", retval);
                return CMOX_ECC_ERR_INTERNAL;
            }

            cmox_ecc_construct(&ecc_ctx, CMOX_ECC256_MATH_FUNCS, working_buffer, sizeof(working_buffer));
            retval = cmox_ecdsa_verify(&ecc_ctx,                                        /* ECC context */
                                       CMOX_ECC_CURVE_SECP256R1,                        /* SECP256R1 ECC curve selected */
                                       compat_params.key, compat_params.key_size,       /* Public key for verification */
                                       compat_params.in, compat_params.in_size,         /* Message to verify */
                                       compat_params.signature, compat_params.sig_size, /* Data buffer to receive signature */
                                       NULL);
            cmox_ecc_cleanup(&ecc_ctx);
            break;

        default:
            retval = CMOX_ECC_ERR_BAD_PARAMETERS;
            break;
    }

    return retval;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static cmox_ecc_retval_t do_ecc_dsa_sign(sid_pal_dsa_params_t * const params)
{
    cmox_ecc_retval_t retval = CMOX_ECC_ERR_INTERNAL;
    cmox_hash_retval_t hretval;
    uint8_t hash_digest[CMOX_SHA256_SIZE];
    sid_pal_dsa_params_t compat_params;
    uint8_t random_data[CMOX_SHA256_SIZE];
    sid_error_t rand_status;
    uint8_t eddsa_priv_key[CMOX_ECC_ED25519_PRIVKEY_LEN];
    size_t prk_size = 0;
    size_t puk_size = 0;

    switch (params->algo)
    {
        case SID_PAL_EDDSA_ED25519:
            cmox_ecc_construct(&ecc_ctx, CMOX_ECC256_MATH_FUNCS, working_buffer, sizeof(working_buffer));
                                     
            /* Data buffer to receive signature */
            if ( (CMOX_ECC_ED25519_PRIVKEY_LEN - CMOX_ECC_ED25519_PUBKEY_LEN) == params->key_size)
            {
                /* Calculate public key */
                uint8_t * p_out_prk = (uint8_t*)eddsa_priv_key;
                uint8_t * p_out_puk = (uint8_t*)eddsa_priv_key + (CMOX_ECC_ED25519_PRIVKEY_LEN - CMOX_ECC_ED25519_PUBKEY_LEN);
                retval = cmox_eddsa_keyGen(&ecc_ctx,
                                           CMOX_ECC_CURVE_ED25519,
                                           params->key, CMOX_ECC_ED25519_PRIVKEY_LEN - CMOX_ECC_ED25519_PUBKEY_LEN,
                                           p_out_prk, &prk_size,
                                           p_out_puk, &puk_size);
            }
            else if (CMOX_ECC_ED25519_PRIVKEY_LEN == params->key_size)
            {
                /* Copy privkey and pubkey to buffer */
                memcpy(eddsa_priv_key, params->key, CMOX_ECC_ED25519_PRIVKEY_LEN);
                retval = CMOX_ECC_SUCCESS;
            }
            else
            {
                retval = CMOX_ECC_ERR_BAD_PARAMETERS;
            }

            if(CMOX_ECC_SUCCESS == retval)
            {
                retval = cmox_eddsa_sign(&ecc_ctx,                                      /* ECC context */
                                         CMOX_ECC_CURVE_ED25519,                        /* ED25519 ECC curve selected */
                                         eddsa_priv_key, CMOX_ECC_ED25519_PRIVKEY_LEN,  /* Private key for signature */
                                         params->in, params->in_size,                   /* Message to sign */
                                         params->signature, &params->sig_size); 
            }
            
            cmox_ecc_cleanup(&ecc_ctx);
            break;

        case SID_PAL_ECDSA_SECP256R1:
            hretval = get_hash_ecc_dsa_input(params, &compat_params, hash_digest, sizeof(hash_digest));
            if (hretval != CMOX_HASH_SUCCESS)
            {
                SID_PAL_LOG_ERROR("Hash calculation. CMOX err: 0x%08x", retval);
                return CMOX_ECC_ERR_INTERNAL;
            }

            do
            {
                rand_status = stm32wba_crypto_rand(random_data, sizeof(random_data));
                if (rand_status != SID_ERROR_NONE)
                {
                    SID_PAL_LOG_ERROR("Random data creation failed. Error code: %d", rand_status);
                    return CMOX_ECC_ERR_INTERNAL;
                }

                cmox_ecc_construct(&ecc_ctx, CMOX_ECC256_MATH_FUNCS, working_buffer, sizeof(working_buffer));
                retval = cmox_ecdsa_sign(&ecc_ctx,                                          /* ECC context */
                                         CMOX_ECC_CURVE_SECP256R1,                          /* SECP256R1 ECC curve selected */
                                         random_data, sizeof(random_data),                  /* Random data buffer */
                                         compat_params.key, compat_params.key_size,         /* Private key for signature */
                                         compat_params.in, compat_params.in_size,           /* Message to sign */
                                         compat_params.signature, &compat_params.sig_size); /* Data buffer to receive signature */
                cmox_ecc_cleanup(&ecc_ctx);
            } while (CMOX_ECC_ERR_WRONG_RANDOM == retval);
            break;

        default:
            retval = CMOX_ECC_ERR_BAD_PARAMETERS;
            break;
    }

    return retval;
}

/*----------------------------------------------------------------------------*/

/**
  * @brief      Generate private and public keys to use with x25519
  * @param[in]  P_pEccCtx       Context for ECC operations
  * @param[in]  P_pRandom       Buffer of random bytes
  * @param[in]  P_RandomLen     Byte length of the random buffer
  * @param[out] P_pPrivKey      Buffer with the private key
  * @param[out] P_pPrivKeyLen   Byte length of the private key
  * @param[out] P_pPubKey       Buffer with the public key
  * @param[out] P_pPubKeyLen    Byte length of the public key
  * @retval     CMOX_ECC_SUCCESS                Everything OK
  * @retval     CMOX_ECC_ERR_INTERNAL           Systematic code failure, something is wrong with the implementation
  * @retval     CMOX_ECC_ERR_MATHCURVE_MISMATCH Mathematical function set is not compatible with current ECC curve
  * @retval     CMOX_ECC_ERR_BAD_PARAMETERS     Some NULL/wrong/empty parameter or Construct API not called
  * @retval     CMOX_ECC_ERR_WRONG_RANDOM       Random material too short or not valid for the functionality
  * @retval     CMOX_ECC_ERR_MEMORY_FAIL        Not enough memory
  */
SID_STM32_SPEED_OPTIMIZED static cmox_ecc_retval_t do_x25519_keyGen(cmox_ecc_handle_t * const P_pEccCtx,
                                                                   const uint8_t * const     P_pRandom,
                                                                   const size_t              P_RandomLen,
                                                                   uint8_t * const           P_pPrivKey,
                                                                   size_t * const            P_pPrivKeyLen,
                                                                   uint8_t * const           P_pPubKey,
                                                                   size_t * const            P_pPubKeyLen)
{
    cmox_ecc_retval_t retval = CMOX_ECC_ERR_INTERNAL;
    bool wrong_random;

    /* Validate pointers */
    if (NULL == P_pEccCtx)
    {
        retval = CMOX_ECC_ERR_BAD_PARAMETERS;
        goto exit;
    }

    if (NULL == P_pRandom)
    {
        retval = CMOX_ECC_ERR_BAD_PARAMETERS;
        goto exit;
    }

    if ((NULL == P_pPrivKey) || (NULL == P_pPrivKeyLen))
    {
        retval = CMOX_ECC_ERR_BAD_PARAMETERS;
        goto exit;
    }

    if ((NULL == P_pPubKey) || (NULL == P_pPubKeyLen))
    {
        retval = CMOX_ECC_ERR_BAD_PARAMETERS;
        goto exit;
    }

    /* Ensure the random data has correct length */
    if (P_RandomLen != CMOX_ECC_CURVE25519_PRIVKEY_LEN)
    {
        retval = CMOX_ECC_ERR_WRONG_RANDOM;
        goto exit;
    }

    /* Check if random data consists of all zeros */
    wrong_random = true;
    for(size_t i = 0u; i < P_RandomLen; i++)
    {
        if (P_pRandom[i] != 0x00u)
        {
            wrong_random = false;
            break;
        }
    }
    if (wrong_random != false)
    {
        retval = CMOX_ECC_ERR_WRONG_RANDOM;
        goto exit;
    }

    /* Check if random data consists of all ones */
    wrong_random = true;
    for(size_t i = 0u; i < P_RandomLen; i++)
    {
        if (P_pRandom[i] != 0xFFu)
        {
            wrong_random = false;
            break;
        }
    }
    if (wrong_random != false)
    {
        retval = CMOX_ECC_ERR_WRONG_RANDOM;
        goto exit;
    }

    /* Now we can proceed with key generation procedure */

    /* Do x25519 clamping of the random data to obtain the private key */
    memcpy(P_pPrivKey, P_pRandom, P_RandomLen);
    P_pPrivKey[0] &= 0xF8u;
    P_pPrivKey[P_RandomLen - 1] &= 0x7Fu;
    P_pPrivKey[P_RandomLen - 1] |= 0x40u;
    *P_pPrivKeyLen = P_RandomLen;

    /* Now compute the public key */
    retval = cmox_ecdh(&ecc_ctx,                                     /* ECC context */
                       CMOX_ECC_CURVE25519,                          /* CURVE25519 ECC curve selected */
                       P_pPrivKey, *P_pPrivKeyLen,                   /* Local Private key */
                       x25519_base_point, sizeof(x25519_base_point), /* Remote Public key */
                       P_pPubKey, P_pPubKeyLen);                     /* Data buffer to receive shared secret */

exit:
    return retval;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static cmox_ecc_retval_t do_ecc_key_gen(sid_pal_ecc_key_gen_params_t * const params)
{
    cmox_ecc_retval_t retval = CMOX_ECC_ERR_INTERNAL;
    sid_error_t rand_status;

    switch (params->algo)
    {
        case SID_PAL_EDDSA_ED25519:
            {
                uint8_t random_data[CMOX_ECC_ED25519_PRIVKEY_LEN - CMOX_ECC_ED25519_PUBKEY_LEN];

                do
                {
                    rand_status = stm32wba_crypto_rand(random_data, sizeof(random_data));
                    if (rand_status != SID_ERROR_NONE)
                    {
                        retval = CMOX_ECC_ERR_INTERNAL;
                        goto exit;
                    }

                    stm32wba_crypto_obtain_hw_control();
                    cmox_ecc_construct(&ecc_ctx, CMOX_ECC256_MATH_FUNCS, working_buffer, sizeof(working_buffer));
                    retval = cmox_eddsa_keyGen(&ecc_ctx,
                                               CMOX_ECC_CURVE_ED25519,
                                               random_data, sizeof(random_data),
                                               params->prk, &(params->prk_size),
                                               params->puk, &(params->puk_size));
                    cmox_ecc_cleanup(&ecc_ctx);
                    stm32wba_crypto_release_hw_control();
                } while (CMOX_ECC_ERR_WRONG_RANDOM == retval);
            }
            break;

        case SID_PAL_ECDSA_SECP256R1:
        case SID_PAL_ECDH_SECP256R1:
            {
                uint8_t random_data[CMOX_ECC_SECP256R1_PRIVKEY_LEN];

                do
                {
                    rand_status = stm32wba_crypto_rand(random_data, sizeof(random_data));
                    if (rand_status != SID_ERROR_NONE)
                    {
                        retval = CMOX_ECC_ERR_INTERNAL;
                        goto exit;
                    }

                    stm32wba_crypto_obtain_hw_control();
                    cmox_ecc_construct(&ecc_ctx, CMOX_ECC256_MATH_FUNCS, working_buffer, sizeof(working_buffer));
                    retval = cmox_ecdsa_keyGen(&ecc_ctx,
                                               CMOX_ECC_CURVE_SECP256R1,
                                               random_data, sizeof(random_data),
                                               params->prk, &(params->prk_size),
                                               params->puk, &(params->puk_size));
                    cmox_ecc_cleanup(&ecc_ctx);
                    stm32wba_crypto_release_hw_control();
                } while (CMOX_ECC_ERR_WRONG_RANDOM == retval);
            }
            break;

        case SID_PAL_ECDH_CURVE25519:
            {
                uint8_t random_data[CMOX_ECC_CURVE25519_PRIVKEY_LEN];

                do
                {
                    rand_status = stm32wba_crypto_rand(random_data, sizeof(random_data));
                    if (rand_status != SID_ERROR_NONE)
                    {
                        retval = CMOX_ECC_ERR_INTERNAL;
                        goto exit;
                    }

                    stm32wba_crypto_obtain_hw_control();
                    cmox_ecc_construct(&ecc_ctx, CMOX_ECC256_MATH_FUNCS, working_buffer, sizeof(working_buffer));
                    retval = do_x25519_keyGen(&ecc_ctx,
                                              random_data, sizeof(random_data),
                                              params->prk, &(params->prk_size),
                                              params->puk, &(params->puk_size));
                    cmox_ecc_cleanup(&ecc_ctx);
                    stm32wba_crypto_release_hw_control();
                } while (CMOX_ECC_ERR_WRONG_RANDOM == retval);
            }
            break;

        default:
            retval = CMOX_ECC_ERR_BAD_PARAMETERS;
            break;
    }

exit:
    return retval;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t stm32wba_crypto_ecc_dsa(sid_pal_dsa_params_t * const params)
{
    sid_error_t status = SID_ERROR_GENERIC;
    cmox_ecc_retval_t retval = CMOX_ECC_ERR_INTERNAL;

    switch (params->algo)
    {
        case SID_PAL_EDDSA_ED25519:
            if (SID_PAL_CRYPTO_VERIFY == params->mode)
            {
                stm32wba_crypto_obtain_hw_control();
                retval = do_ecc_dsa_verify(params);
                stm32wba_crypto_release_hw_control();

                if (retval != CMOX_ECC_AUTH_SUCCESS)
                {
                    SID_PAL_LOG_ERROR("ECC verification ED25519. CMOX err: 0x%08x", retval);
                    status = SID_ERROR_GENERIC;
                }
                else
                {
                    status = SID_ERROR_NONE;
                }
            }
            else if (SID_PAL_CRYPTO_SIGN == params->mode)
            {
                stm32wba_crypto_obtain_hw_control();
                retval = do_ecc_dsa_sign(params);
                stm32wba_crypto_release_hw_control();

                if (retval != CMOX_ECC_SUCCESS)
                {
                    SID_PAL_LOG_ERROR("ECC signing ED25519. CMOX err: 0x%08x", retval);
                    status = SID_ERROR_GENERIC;
                }
                else
                {
                    status = SID_ERROR_NONE;
                }
            }
            else
            {
                status = SID_ERROR_INVALID_ARGS;
            }
            break;

        case SID_PAL_ECDSA_SECP256R1:
            if (SID_PAL_CRYPTO_VERIFY == params->mode)
            {
                stm32wba_crypto_obtain_hw_control();
                retval = do_ecc_dsa_verify(params);
                stm32wba_crypto_release_hw_control();

                if (retval != CMOX_ECC_AUTH_SUCCESS)
                {
                    SID_PAL_LOG_ERROR("ECC verification SECP256R1. CMOX err: 0x%08x", retval);
                    status = SID_ERROR_GENERIC;
                }
                else
                {
                    status = SID_ERROR_NONE;
                }
            }
            else if (SID_PAL_CRYPTO_SIGN == params->mode)
            {
                stm32wba_crypto_obtain_hw_control();
                retval = do_ecc_dsa_sign(params);
                stm32wba_crypto_release_hw_control();

                if (retval != CMOX_ECC_SUCCESS)
                {
                    SID_PAL_LOG_ERROR("ECC signing SECP256R1. CMOX err: 0x%08x", retval);
                    status = SID_ERROR_GENERIC;
                }
                else
                {
                    status = SID_ERROR_NONE;
                }
            }
            else
            {
                status = SID_ERROR_INVALID_ARGS;
            }
            break;

        default:
            status = SID_ERROR_NOSUPPORT;
    }

    return status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t stm32wba_crypto_ecc_ecdh(sid_pal_ecdh_params_t * const params)
{
    sid_error_t status = SID_ERROR_GENERIC;
    cmox_ecc_retval_t retval = CMOX_ECC_ERR_INTERNAL;

    switch (params->algo)
    {
        case SID_PAL_ECDH_SECP256R1:
            {
                uint8_t computed_secret[CMOX_ECC_SECP256R1_SECRET_LEN];
                size_t computed_secret_sz;
                const size_t useful_secret_bytes = CMOX_ECC_SECP256R1_SECRET_LEN / 2u;

                /* Get xclusive access to CRC peripheral and confgure it properly */
                stm32wba_crypto_obtain_hw_control();

                /* Construct a ECC context, specifying mathematics implementation and working buffer for later processing */
                /* Note: CMOX_ECC256_MATH_FUNCS refer to the default mathematics implementation
                 * selected in cmox_default_config.h. To use a specific implementation, user can
                 * directly choose:
                 * - CMOX_MATH_FUNCS_SMALL to select the mathematics small implementation
                 * - CMOX_MATH_FUNCS_FAST to select the mathematics fast implementation
                 * - CMOX_MATH_FUNCS_SUPERFAST256 to select the mathematics fast implementation optimized for 256 bits computation
                 */
                cmox_ecc_construct(&ecc_ctx, CMOX_ECC256_MATH_FUNCS, working_buffer, sizeof(working_buffer));

                /* Compute directly the shared secret passing all the needed parameters */
                /* Note: CMOX_ECC_CURVE_SECP256R1 refer to the default SECP256R1 definition
                 * selected in cmox_default_config.h. To use a specific definition, user can
                 * directly choose:
                 * - CMOX_ECC_SECP256R1_LOWMEM to select the low RAM usage definition (slower computing)
                 * - CMOX_ECC_SECP256R1_HIGHMEM to select the high RAM usage definition (faster computing)
                 */
                retval = cmox_ecdh(&ecc_ctx,                              /* ECC context */
                                   CMOX_ECC_CURVE_SECP256R1,              /* SECP256R1 ECC curve selected */
                                   params->prk, params->prk_size,         /* Local Private key */
                                   params->puk, params->puk_size,         /* Remote Public key */
                                   computed_secret, &computed_secret_sz); /* Data buffer to receive shared secret */

                /* Cleanup context */
                cmox_ecc_cleanup(&ecc_ctx);

                /* Restore CRC peripheral state and release the lock */
                stm32wba_crypto_release_hw_control();
                if (retval != CMOX_ECC_SUCCESS)
                {
                    SID_PAL_LOG_ERROR("Failed to compute SECP256R1 ECC. CMOX error code: %08x", retval);
                    status = SID_ERROR_IO_ERROR;
                }
                else
                {
                    SID_PAL_ASSERT(CMOX_ECC_SECP256R1_SECRET_LEN == computed_secret_sz);
                    memcpy(params->shared_secret, computed_secret, useful_secret_bytes);
                    params->shared_secret_sz = useful_secret_bytes;
                    status = SID_ERROR_NONE;
                }
            }
            break;

        case SID_PAL_ECDH_CURVE25519:
            stm32wba_crypto_obtain_hw_control();
            cmox_ecc_construct(&ecc_ctx, CMOX_ECC256_MATH_FUNCS, working_buffer, sizeof(working_buffer));
            retval = cmox_ecdh(&ecc_ctx,                                          /* ECC context */
                               CMOX_ECC_CURVE25519,                               /* CURVE25519 ECC curve selected */
                               params->prk, params->prk_size,                     /* Local Private key */
                               params->puk, params->puk_size,                     /* Remote Public key */
                               params->shared_secret, &params->shared_secret_sz); /* Data buffer to receive shared secret */
            cmox_ecc_cleanup(&ecc_ctx);
            stm32wba_crypto_release_hw_control();

            if (retval != CMOX_ECC_SUCCESS)
            {
                SID_PAL_LOG_ERROR("Failed to compute CURVE25519 ECC. CMOX error code: %08x", retval);
                status = SID_ERROR_IO_ERROR;
            }
            else
            {
                status = SID_ERROR_NONE;
            }
            break;

        default:
            retval = CMOX_ECC_ERR_BAD_PARAMETERS;
            status = SID_ERROR_NOSUPPORT;
    }

    return status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t stm32wba_crypto_ecc_key_gen(sid_pal_ecc_key_gen_params_t * const params)
{
    sid_error_t status = SID_ERROR_GENERIC;
    cmox_ecc_retval_t retval;
    size_t ecc_prk_sz = 0u, ecc_puk_sz = 0u;

    switch (params->algo)
    {
        case SID_PAL_EDDSA_ED25519:
            ecc_prk_sz = CMOX_ECC_ED25519_PRIVKEY_LEN - CMOX_ECC_ED25519_PUBKEY_LEN; /* Store only private part of the key pair */
            ecc_puk_sz = CMOX_ECC_ED25519_PUBKEY_LEN;
            break;

        case SID_PAL_ECDSA_SECP256R1:
        case SID_PAL_ECDH_SECP256R1:
            ecc_prk_sz = CMOX_ECC_SECP256R1_PRIVKEY_LEN;
            ecc_puk_sz = CMOX_ECC_SECP256R1_PUBKEY_LEN;
        break;

        case SID_PAL_ECDH_CURVE25519:
            ecc_prk_sz = CMOX_ECC_CURVE25519_PRIVKEY_LEN;
            ecc_puk_sz = CMOX_ECC_CURVE25519_PUBKEY_LEN;
            break;

        default:
            return SID_ERROR_NOSUPPORT;
    }

    if ((ecc_prk_sz != params->prk_size) || (ecc_puk_sz != params->puk_size))
    {
        return SID_ERROR_INVALID_ARGS;
    }

    retval = do_ecc_key_gen(params);
    if (retval != CMOX_ECC_SUCCESS)
    {
        SID_PAL_LOG_ERROR("Unable to generate crypto key pair. CMOX error: 0x%08x", retval);
        status = SID_ERROR_IO_ERROR;
    }
    else
    {
        status = SID_ERROR_NONE;
    }

    return status;
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_crypto_init()
{
    if (hal_init_done != false)
    {
        return SID_ERROR_NONE;
    }

    sid_error_t ret = stm32wba_crypto_init();
    if (ret != SID_ERROR_NONE)
    {
        return ret;
    }

    hal_init_done = true;

    uint32_t seed = 0u;
    ret = sid_pal_crypto_rand((uint8_t*)&seed, sizeof(seed));
    if (SID_ERROR_NONE == ret)
    {
        srand(seed);
    }
    else
    {
        hal_init_done = false;
    }

    return ret;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_crypto_deinit(void)
{
    sid_error_t ret = stm32wba_crypto_deinit();
    if (ret != SID_ERROR_NONE)
    {
        return ret;
    }
    else
    {
        hal_init_done = false;
        return SID_ERROR_NONE;
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_crypto_rand(uint8_t* rand, size_t size)
{
    if (!hal_init_done)
    {
        return SID_ERROR_UNINITIALIZED;
    }

    if ((NULL == rand) || (0u == size))
    {
        return SID_ERROR_NULL_POINTER;
    }

    return stm32wba_crypto_rand(rand, size);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_crypto_hash(sid_pal_hash_params_t* params)
{
    cmox_hash_retval_t retval;

    if (false == hal_init_done)
    {
        return SID_ERROR_UNINITIALIZED;
    }

    if ((NULL == params) || (NULL == params->data) || (NULL == params->digest))
    {
        return SID_ERROR_NULL_POINTER;
    }

    if ((0u == params->data_size) || (0u == params->digest_size))
    {
        return SID_ERROR_INVALID_ARGS;
    }

    retval = stm32wba_crypto_hash(params, true);
    return retval == CMOX_HASH_SUCCESS ? SID_ERROR_NONE : SID_ERROR_GENERIC;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_crypto_hmac(sid_pal_hmac_params_t* params)
{
    if (false == hal_init_done)
    {
        return SID_ERROR_UNINITIALIZED;
    }

    if ((NULL == params) || (NULL == params->key) || (NULL == params->data) || (NULL == params->digest))
    {
        return SID_ERROR_NULL_POINTER;
    }

    return stm32wba_crypto_hmac(params);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_crypto_aes_crypt(sid_pal_aes_params_t* params)
{
    if (false == hal_init_done)
    {
        return SID_ERROR_UNINITIALIZED;
    }

    if ((NULL == params) || (NULL == params->key) || (NULL == params->in) || (NULL == params->out))
    {
        return SID_ERROR_NULL_POINTER;
    }

    if (0u == params->in_size)
    {
        return SID_ERROR_INVALID_ARGS;
    }

    return stm32wba_crypto_aes_crypt(params);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_crypto_aead_crypt(sid_pal_aead_params_t* params)
{
    if (false == hal_init_done)
    {
        return SID_ERROR_UNINITIALIZED;
    }

    if ((NULL == params) || (NULL == params->key) || (NULL == params->in) || (NULL == params->out))
    {
        return SID_ERROR_NULL_POINTER;
    }

    if ((0u == params->in_size) || (0u == params->aad_size))
    {
        return SID_ERROR_INVALID_ARGS;
    }

    return stm32wba_crypto_aead_crypt(params);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_crypto_ecc_dsa(sid_pal_dsa_params_t* params)
{
    if (false == hal_init_done)
    {
        return SID_ERROR_UNINITIALIZED;
    }

    if ((NULL == params) || (NULL == params->key) || (NULL == params->in) || (NULL == params->signature))
    {
        return SID_ERROR_NULL_POINTER;
    }

    if (0u == params->in_size)
    {
        return SID_ERROR_INVALID_ARGS;
    }

    return stm32wba_crypto_ecc_dsa(params);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_crypto_ecc_ecdh(sid_pal_ecdh_params_t* params)
{
    if (false == hal_init_done)
    {
        return SID_ERROR_UNINITIALIZED;
    }

    if ((NULL == params) || (NULL == params->prk) || (NULL == params->puk) || (NULL == params->shared_secret))
    {
        return SID_ERROR_NULL_POINTER;
    }

    return stm32wba_crypto_ecc_ecdh(params);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_crypto_ecc_key_gen(sid_pal_ecc_key_gen_params_t* params)
{
    if (false == hal_init_done)
    {
        return SID_ERROR_UNINITIALIZED;
    }

    if ((NULL == params) || (NULL == params->prk) || (NULL == params->puk))
    {
        return SID_ERROR_NULL_POINTER;
    }

    return stm32wba_crypto_ecc_key_gen(params);
}

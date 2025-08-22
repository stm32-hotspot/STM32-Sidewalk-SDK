<center>

# These are full license terms for
# <mark>STM32-Sidewalk-SDK</mark>

Copyright &copy;2025 STMicroelectronics

All rights reserved
    
</center>

## <mark>__OVERVIEW__</mark>
<div>

This Software Bill Of Material (SBOM) lists the software components of this
software package, including the copyright owner and license terms for each
component.

The full text of these licenses are below the SBOM.

__SOFTWARE BILL OF MATERIALS__

| Component                                                                                  | Copyright                                   | License
|----------                                                                                  |----------                                   |--------
| CMSIS                                                                                      | Arm Limited                                 | [Apache-2.0](https://opensource.org/licenses/Apache-2.0)
| STM32WBAxx CMSIS                                                                           | Arm Limited, STMicroelectronics             | [Apache-2.0](https://opensource.org/licenses/Apache-2.0)
| STM32WLxx CMSIS                                                                            | Arm Limited, STMicroelectronics             | [Apache-2.0](https://opensource.org/licenses/Apache-2.0)
| STM32WBAxx_HAL_Driver                                                                      | STMicroelectronics                          | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| STM32WLxx_HAL_Driver                                                                       | STMicroelectronics                          | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| BSP STM32WBAxx_Nucleo                                                                      | STMicroelectronics                          | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| BSP STM32WLxx_Nucleo                                                                       | STMicroelectronics                          | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| FreeRTOS (Middleware)                                                                      | Amazon.com, Inc. or its affiliates          | [MIT](https://opensource.org/license/MIT)
| STM32_WPAN (Middleware including Bluetooth Low Energy, Linklayer)                          | STMicroelectronics, Synopsys, Inc.          | [SLA](#collapse-section1)                                                                  
| littleFS File System (Middleware)                                                          | The littlefs authors, Arm Limited           | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| SubGHz_Phy (Middleware)                                                                    | Semtech, STMicroelectronics                 | [ANNEX 3](#collapse-section3)
| Utilities                                                                                  | STMicroelectronics                          | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| STM32_Cryptographic (Middleware)                                                           | STMicroelectronics                          | [SLA](#collapse-section1)
| Sidewalk Unified Demo Apps (_apps/common_)                                                 | Amazon.com, Inc. or its affiliates          | [ANNEX 2](#collapse-section2)
| Common STM32 Platform Code (_apps/st/common_)                                              | STMicroelectronics                          | [SLA](#collapse-section1)
| Common STM32WBAxx Platform Code (_apps/st/stm32wba_)                                       | STMicroelectronics                          | [SLA](#collapse-section1)
| Sidewalk Sample Apps for WBAxx (_apps/st/stm32wba/sid_xxx_)                                | STMicroelectronics                          | [SLA](#collapse-section1)
| Sidewalk Sample Apps for WBAxx (_apps/st/stm32wba/sid_xxx_)                                | Amazon.com, Inc. or its affiliates          | [ANNEX 2](#collapse-section2)
| Sidewalk Sample Apps for WBAxx (_apps/st/stm32wba/sid_xxx_)                                | Semtech                                     | [ANNEX 3](#collapse-section3)
| Sidewalk Platform Abstraction Layer (PAL):  Common STM32 PAL (_pal/st/common_)             | STMicroelectronics                          | [SLA](#collapse-section1)
| Sidewalk Platform Abstraction Layer (PAL):  STM32WBAxx PAL (_pal/st/stm32wba_)             | STMicroelectronics                          | [SLA](#collapse-section1)
| Semtech Common Interfaces (_platform/sid_mcu/semtech/hal/common_)                          | Amazon.com, Inc. or its affiliates          | [ANNEX 2](#collapse-section2)
| Semtech LR11xx Sidewalk Driver (_platform/sid_mcu/semtech/hal/lr11xx_)                     | STMicroelectronics                          | [SLA](#collapse-section1)
| Semtech LR11xx Sidewalk Driver (_platform/sid_mcu/semtech/hal/lr11xx_)                     | Amazon.com, Inc. or its affiliates          | [ANNEX 2](#collapse-section2)
| Semtech LR11xx Driver (_platform/sid_mcu/semtech/hal/lr11xx/semtech_)                      | Semtech Corporation, STMicroelectronics     | [ANNEX 3](#collapse-section3)
| Semtech SX126x Sidewalk Driver (_platform/sid_mcu/semtech/hal/sx126x_)                     | STMicroelectronics                          | [SLA](#collapse-section1)
| Semtech SX126x Sidewalk Driver (_platform/sid_mcu/semtech/hal/sx126x_)                     | Amazon.com, Inc. or its affiliates          | [ANNEX 2](#collapse-section2)
| Semtech SX126x Driver (_platform/sid_mcu/semtech/hal/sx126x/semtech_)                      | Semtech Corporation, STMicroelectronics     | [ANNEX 3](#collapse-section3)
| LoRa Basics Modem Middleware (_platform/sid_mcu/semtech/lbm_lib_)                          | Semtech Corporation, STMicroelectronics     | [ANNEX 3](#collapse-section3)
| LoRa Basics Modem Sidewalk Adapter (_platform/sid_mcu/semtech/lbm_sidewalk_adapter_)       | STMicroelectronics                          | [SLA](#collapse-section1)
| Spirit2 (S2-LP) Radio Driver (_platform/sid_mcu/st/hal/s2-lp_)                             | STMicroelectronics                          | [SLA](#collapse-section1)
| STM32WBAxx Sidewalk HAL (_platform/sid_mcu/st/hal/stm32wba_)                               | STMicroelectronics                          | [SLA](#collapse-section1)
| STM32WLxx Radio App Driver for WBAxx (_platform/sid_mcu/st/hal/stm32wlxx_radio_app_)       | STMicroelectronics, Semtech Corporation     | [SLA](#collapse-section1)
| Sidewalk MCU SDK Library and Code (_sidewalk_sdk_prebuilt_)                                | Amazon.com, Inc. or its affiliates          | [ANNEX 2](#collapse-section2)
| nanopb (_sidewalk_sdk_prebuilt_)                                                           | Petteri Aimonen                             | [Zlib](https://opensource.org/license/Zlib)
| LK embedded kernel (_sidewalk_sdk_prebuilt_)                                               | Travis Geiselbrecht                         | [MIT](https://opensource.org/license/MIT)
| uthash (_sidewalk_sdk_prebuilt_)                                                           | Troy D. Hanson                              | [BSD-1-Clause](https://opensource.org/licenses/BSD-1-Clause)
| Sidewalk Provision Tool (_tools/provision_)                                                | Amazon.com, Inc. or its affiliates          | [Apache-2.0](https://opensource.org/licenses/Apache-2.0)

__Notes:__ If the license is an open source license, then you can access the
terms at [www.opensource.org](https://opensource.org/). Otherwise, the full
license terms are below. If a component is not listed in the SBOM, then the SLA
shall apply unless other terms are clearly stated in the package.

</div>

<label id="collapse-section1" aria-hidden="true">__SLA – Software License Agreement__</label>
<div>

SLA0048 Rev4/March 2018

## Software license agreement

### __SOFTWARE PACKAGE LICENSE AGREEMENT__

BY INSTALLING COPYING, DOWNLOADING, ACCESSING OR OTHERWISE USING THIS SOFTWARE PACKAGE OR ANY
PART THEREOF (AND THE RELATED DOCUMENTATION) FROM STMICROELECTRONICS INTERNATIONAL N.V, SWISS
BRANCH AND/OR ITS AFFILIATED COMPANIES (STMICROELECTRONICS), THE RECIPIENT, ON BEHALF OF HIMSELF
OR HERSELF, OR ON BEHALF OF ANY ENTITY BY WHICH SUCH RECIPIENT IS EMPLOYED AND/OR ENGAGED
AGREES TO BE BOUND BY THIS SOFTWARE PACKAGE LICENSE AGREEMENT.

Under STMicroelectronics’ intellectual property rights and subject to applicable licensing terms for any third-party software
incorporated in this software package and applicable Open Source Terms (as defined here below), the redistribution,
reproduction and use in source and binary forms of the software package or any part thereof, with or without modification, are
permitted provided that the following conditions are met:

1. Redistribution of source code (modified or not) must retain any copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form, except as embedded into microcontroller or microprocessor device manufactured by or for
STMicroelectronics or a software update for such device, must reproduce the above copyright notice, this list of conditions
and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of STMicroelectronics nor the names of other contributors to this software package may be used to
endorse or promote products derived from this software package or part thereof without specific written permission.

4. This software package or any part thereof, including modifications and/or derivative works of this software package, must
be used and execute solely and exclusively on or in combination with a microcontroller or a microprocessor devices
manufactured by or for STMicroelectronics.

5. No use, reproduction or redistribution of this software package partially or totally may be done in any manner that would
subject this software package to any Open Source Terms (as defined below).

6. Some portion of the software package may contain software subject to Open Source Terms (as defined below) applicable
for each such portion (“Open Source Software”), as further specified in the software package. Such Open Source Software
is supplied under the applicable Open Source Terms and is not subject to the terms and conditions of license hereunder.
“Open Source Terms” shall mean any open source license which requires as part of distribution of software that the source
code of such software is distributed therewith or otherwise made available, or open source license that substantially
complies with the Open Source definition specified at www.opensource.org and any other comparable open source license
such as for example GNU General Public License (GPL), Eclipse Public License (EPL), Apache Software License, BSD
license and MIT license.

7. This software package may also include third party software as expressly specified in the software package subject to
specific license terms from such third parties. Such third party software is supplied under such specific license terms and is
not subject to the terms and conditions of license hereunder. By installing copying, downloading, accessing or otherwise
using this software package, the recipient agrees to be bound by such license terms with regard to such third party
software.

8. STMicroelectronics has no obligation to provide any maintenance, support or updates for the software package.

9. The software package is and will remain the exclusive property of STMicroelectronics and its licensors. The recipient will
not take any action that jeopardizes STMicroelectronics and its licensors' proprietary rights or acquire any rights in the
software package, except the limited rights specified hereunder.

10. The recipient shall comply with all applicable laws and regulations affecting the use of the software package or any part
thereof including any applicable export control law or regulation.

11. Redistribution and use of this software package partially or any part thereof other than as permitted under this license is
void and will automatically terminate your rights under this license.

THIS SOFTWARE PACKAGE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY
INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE PACKAGE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

EXCEPT AS EXPRESSLY PERMITTED HEREUNDER AND SUBJECT TO THE APPLICABLE LICENSING TERMS FOR ANY
THIRD-PARTY SOFTWARE INCORPORATED IN THE SOFTWARE PACKAGE AND OPEN SOURCE TERMS AS
APPLICABLE, NO LICENSE OR OTHER RIGHTS, WHETHER EXPRESS OR IMPLIED, ARE GRANTED UNDER ANY
PATENT OR OTHER INTELLECTUAL PROPERTY RIGHTS OF STMICROELECTRONICS OR ANY THIRD PARTY.

</div>


<label id="collapse-section2" aria-hidden="true">__ANNEX 2 - AMAZON SOFTWARE LICENSE TERMS__</label>
<div>

Amazon Sidewalk Content License

©2023 Amazon.com, Inc. or its affiliates (collectively, “Amazon”). All Rights Reserved.

These materials are licensed to you as "AWS Content" under the AWS Intellectual Property License (https://aws.amazon.com/legal/aws-ip-license-terms/) (the “IP License”) and made available to you solely in connection with the developer program for Amazon Sidewalk and related services (such AWS Content, “Amazon Sidewalk Content”). 

Capitalized terms not defined in this file have the meanings given to them in the AWS Customer Agreement (https://aws.amazon.com/agreement/) or the Service Terms (https://aws.amazon.com/service-terms/), including any documentation referenced or incorporated therein (collectively, the “Agreement”).

In addition to the terms and conditions of the IP License, the following terms and conditions ("Additional Terms," and collectively with the IP License, the "Terms") apply to the Amazon Sidewalk Content. In the event of a conflict between the IP License and these Additional Terms, the IP License will control.

You may only use the Amazon Sidewalk Content to (a) evaluate, develop, test, and troubleshoot the operation and compatibility of the Amazon Sidewalk Content with your AS Devices; and (b) integrate the Amazon Sidewalk Content into your AS Devices that access Amazon Sidewalk. If any of the Amazon Sidewalk Content incorporated in your AS Device were provided to you in source code form, such Amazon Sidewalk Content (and modifications thereof) may be distributed solely in binary form as incorporated in your AS Devices. You may not distribute any of your AS Devices that incorporate or were developed using the Amazon Sidewalk Content unless you submit your AS Device to us for approval and we specifically approve the distribution in writing.  You may not: (i) use Amazon Sidewalk Content with any software or other materials that are subject to licenses or restrictions (e.g., open source software licenses) that, when combined with Amazon Sidewalk Content, would require you or us to disclose, license, distribute, or otherwise make all or any part of such Amazon Sidewalk Content available to anyone; or (ii) remove, modify, or obscure any copyright, patent, trademark, or other proprietary or attribution notices on or in any Amazon Sidewalk Content.

Notwithstanding the foregoing, if you are a system-on-chip manufacturer, you may distribute the Amazon Sidewalk Content, including any source code (subject to the limitations herein) made available by Amazon, to AS Device manufacturers for the purpose of such AS Device manufacturers integrating the Amazon Sidewalk Content in their AS Devices and their associated applications and services; provided that you also make available this LICENSE.TXT file to such device manufacturer together with the Amazon Sidewalk Content. For the purpose of the Terms, such Amazon Sidewalk Content will be deemed to have been made available to the device manufacturers from Amazon.

WITH THE EXCEPTION OF FILES IDENTIFIED AS A "MODIFIABLE FILE" WITHIN THE FILE HEADER, YOU MAY NOT MODIFY ANY SOURCE CODE IN ANY FILE IN THIS PACKAGE. Your modifications to the Amazon Sidewalk Content are “Modified Content”. You may reproduce and distribute copies of the Modified Content thereof in any medium, with or without modifications, and in source or object form, provided that you meet the following conditions:

* You must give any other recipients of the Modified Content a copy of these Terms; and

* You must cause any modified files to carry prominent notices stating that you changed the files; and

* You must retain, in the source form of any Modified Content that you distribute, all copyright, patent, trademark, and attribution notices from the source form of the Amazon Sidewalk Content, excluding those notices that do not pertain to any part of the Modified Content; and

* If the Amazon Sidewalk Content includes a "NOTICE" text file as part of its distribution, then any Modified Content that you distribute must include a readable copy of the attribution notices contained within such NOTICE file, excluding those notices that do not pertain to any part of the Modified Content, in at least one of the following places: within a NOTICE text file distributed as part of the Modified Content; within the source form or documentation, if provided along with the Modified Content; or, within a display generated by the Modified Content, if and wherever such third-party notices normally appear. The contents of the NOTICE file are for informational purposes only and do not modify the Terms. You may add your own attribution notices within Modified Content that you distribute, alongside or as an addendum to the NOTICE text from the Amazon Sidewalk Content, provided that such additional attribution notices cannot be construed as modifying these Terms.

You may add your own copyright statement to your modifications and may provide additional or different license terms and conditions for use, reproduction, or distribution of your modifications, or for any such Modified Content as a whole, provided your use, reproduction, and distribution of the Amazon Sidewalk Content otherwise complies with the conditions stated in these Terms.

Our licensors may enforce the Terms against you with respect to their software and other materials included in the Amazon Sidewalk Content, and our licensors are third-party beneficiaries of these Terms solely for that purpose.

Amazon Sidewalk Content may include and/or be dependent upon certain third-party libraries or other software packages ("External Dependencies").  If you do not agree with every term in the license file associated with the External Dependencies, you should not use these materials.

THE AMAZON SIDEWALK CONTENT AND THE EXTERNAL DEPENDENCIES ARE PROVIDED BY AMAZON AND AMAZON’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. AMAZON DOES NOT PROMISE THAT THE EXTERNAL DEPENDENCIES AND THEIR APPLICABLE TERMS ARE COMPLETE, ACCURATE, OR UP-TO-DATE, AND AMAZON WILL HAVE NO LIABILITY FOR ANY OMISSIONS OR INACCURACIES. YOU SHOULD CONSULT THE DOWNLOAD SITES FOR THE EXTERNAL DEPENDENCIES FOR THE MOST COMPLETE AND UP-TO-DATE LICENSING INFORMATION. YOUR USE OF THE AMAZON SIDEWALK CONTENT AND THE EXTERNAL DEPENDENCIES IS AT YOUR SOLE RISK. IN NO EVENT WILL AMAZON BE LIABLE FOR ANY DAMAGES, INCLUDING WITHOUT LIMITATION ANY DIRECT, INDIRECT, CONSEQUENTIAL, SPECIAL, INCIDENTAL, OR PUNITIVE DAMAGES (INCLUDING FOR ANY LOSS OF GOODWILL, BUSINESS INTERRUPTION, LOST PROFITS OR DATA, OR COMPUTER FAILURE OR MALFUNCTION) ARISING FROM OR RELATING TO THE AMAZON SIDEWALK CONTENT OR EXTERNAL DEPENDENCIES, HOWEVER CAUSED AND REGARDLESS OF THE THEORY OF LIABILITY, EVEN IF AMAZON HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES. THESE LIMITATIONS AND DISCLAIMERS APPLY EXCEPT TO THE EXTENT PROHIBITED BY APPLICABLE LAW.

You will defend, indemnify, and hold harmless Amazon, its affiliates, and licensors against any and all claims, losses, and damages arising out of or relating to your distribution of the Amazon Sidewalk Content or External Dependencies in breach of the Terms.

If you do not agree to the Terms, you may not use any file in this package.

</div>


<label id="collapse-section3" aria-hidden="true">__ANNEX 3 - The Clear BSD License__</label>
<div>

The Clear BSD License
Copyright Semtech Corporation 2023. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the Semtech corporation nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

</div>

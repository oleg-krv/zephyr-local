# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(
  bt.c
  hci.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_CONN
  gatt.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_BREDR
  bredr.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_L2CAP_DYNAMIC_CHANNEL
  l2cap.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_RFCOMM
  rfcomm.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_ISO
  iso.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_VCS
  vcs.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_VCS_CLIENT
  vcs_client.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_MICS
  mics.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_MICS_CLIENT
  mics_client.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_CSIS
  csis.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_CSIS_CLIENT
  csis_client.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_MPL
  mpl.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_MCC
  mcc.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_MCS
  media_controller.c
  )
# We use BT_AUDIO_STREAM as a common ground for audio, as that is set whenever
# any audio stream functionality is enabled.
zephyr_library_sources_ifdef(
  CONFIG_BT_AUDIO_STREAM
  audio.c
  )

zephyr_library_sources_ifdef(
  CONFIG_BT_BASS
  bass.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_BASS_CLIENT
  bass_client.c
  )

if(CONFIG_BT_CTLR AND CONFIG_BT_LL_SW_SPLIT)
  zephyr_library_sources(
    ll.c
    ticker.c
    )
  zephyr_include_directories(
    ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic
    )
endif()

# Downloads only the SO-101 files required by the simulator. Each file is
# checked against the pinned MuJoCo Menagerie revision before it is used.
function(amp_fetch_so101_model output_variable)
  set(_revision "8161bba264d7fa7c99ca301e91e7fb44737676ad")
  set(_base_url "https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/${_revision}/robotstudio_so101")
  set(_destination "${CMAKE_BINARY_DIR}/_deps/robotstudio_so101")

  set(_manifest
    "LICENSE" "c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4"
    "scene.xml" "d3037f0fd36b61a9b6805b2c79831e7a392276d0f490145d01f718542eee9150"
    "so101.xml" "5ad49f2b45c083baac9ffe5d4d3213a5da7eac8039095bb2df177a697aae8308"
    "assets/base_motor_holder_so101_v1.stl" "8cd2f241037ea377af1191fffe0dd9d9006beea6dcc48543660ed41647072424"
    "assets/base_so101_v2.stl" "bb12b7026575e1f70ccc7240051f9d943553bf34e5128537de6cd86fae33924d"
    "assets/motor_holder_so101_base_v1.stl" "31242ae6fb59d8b15c66617b88ad8e9bded62d57c35d11c0c43a70d2f4caa95b"
    "assets/motor_holder_so101_wrist_v1.stl" "887f92e6013cb64ea3a1ab8675e92da1e0beacfd5e001f972523540545e08011"
    "assets/moving_jaw_so101_gripper_part0_v1.stl" "3a92e115d1934d4c45cff808c4eff2ea10361a137a7f89940033a1f8a821b88c"
    "assets/moving_jaw_so101_gripper_part1_v1.stl" "d10f6c671d26f62988ec95efee9370a0657b5833b3c5f8aad59fd69a87ef2aee"
    "assets/moving_jaw_so101_gripper_v1.stl" "e8a5dc040b98e27453a5b74f9087d1dbca14589cf9a17c4de7868d5ab98413cd"
    "assets/moving_jaw_so101_v1.stl" "785a9dded2f474bc1d869e0d3dae398a3dcd9c0c345640040472210d2861fa9d"
    "assets/rotation_pitch_so101_v1.stl" "9be900cc2a2bf718102841ef82ef8d2873842427648092c8ed2ca1e2ef4ffa34"
    "assets/sts3215_03a_no_horn_v1.stl" "75ef3781b752e4065891aea855e34dc161a38a549549cd0970cedd07eae6f887"
    "assets/sts3215_03a_v1.stl" "a37c871fb502483ab96c256baf457d36f2e97afc9205313d9c5ab275ef941cd0"
    "assets/under_arm_so101_v1.stl" "d01d1f2de365651dcad9d6669e94ff87ff7652b5bb2d10752a66a456a86dbc71"
    "assets/upper_arm_so101_v1.stl" "475056e03a17e71919b82fd88ab9a0b898ab50164f2a7943652a6b2941bb2d4f"
    "assets/waveshare_mounting_plate_so101_v2.stl" "e197e24005a07d01bbc06a8c42311664eaeda415bf859f68fa247884d0f1a6e9"
    "assets/wrist_roll_follower_so101_camera_mount.stl" "58c457b33a26d9c9bb337aa351173e7123f84719a93f486541adef6eba57ae21"
    "assets/wrist_roll_follower_so101_gripper_part0_v1.stl" "370eb470a6ab48b08febcaeb5274d2bcc91246692532ce12c795fb82977def72"
    "assets/wrist_roll_follower_so101_gripper_v1.stl" "65f32debcf265420c0160afe004f8befc5df4b3e578fd59ebaa62eade0371224"
    "assets/wrist_roll_follower_so101_v1.stl" "4b17b410a12d64ec39554abc3e8054d8a97384b2dc4a8d95a5ecb2a93670f5f4"
    "assets/wrist_roll_pitch_so101_v2.stl" "6c7ec5525b4d8b9e397a30ab4bb0037156a5d5f38a4adf2c7d943d6c56eda5ae"
  )

  list(LENGTH _manifest _manifest_length)
  math(EXPR _last_index "${_manifest_length} - 2")
  foreach(_index RANGE 0 ${_last_index} 2)
    math(EXPR _hash_index "${_index} + 1")
    list(GET _manifest ${_index} _relative_path)
    list(GET _manifest ${_hash_index} _sha256)
    set(_local_path "${_destination}/${_relative_path}")

    set(_download TRUE)
    if(EXISTS "${_local_path}")
      file(SHA256 "${_local_path}" _actual_sha256)
      if(_actual_sha256 STREQUAL _sha256)
        set(_download FALSE)
      endif()
    endif()

    if(_download)
      get_filename_component(_local_directory "${_local_path}" DIRECTORY)
      file(MAKE_DIRECTORY "${_local_directory}")
      message(STATUS "Downloading SO-101 model file: ${_relative_path}")
      file(
        DOWNLOAD "${_base_url}/${_relative_path}" "${_local_path}"
        EXPECTED_HASH "SHA256=${_sha256}"
        TLS_VERIFY ON
        STATUS _download_status
      )
      list(GET _download_status 0 _download_code)
      list(GET _download_status 1 _download_message)
      if(NOT _download_code EQUAL 0)
        message(FATAL_ERROR "Could not download ${_relative_path}: ${_download_message}")
      endif()
    endif()
  endforeach()

  set(${output_variable} "${_destination}" PARENT_SCOPE)
endfunction()


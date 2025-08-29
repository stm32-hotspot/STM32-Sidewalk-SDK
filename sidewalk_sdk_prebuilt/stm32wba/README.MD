# Sidewalk Stack Library Variants

This folder contains precompiled Sidewalk SDK using different configurations. Below are some hints on each library

- `sidewalk_sdk_basic` - regular Sidewalk stack with reduced feature set to achieve smaller flash & RAM footprints
- `sidewalk_sdk_full` - regular Sidewalk stack with all features included and configurable in runtime
- `sidewalk_dut` - Amazon's proprietary application code for `sid_dut` application
- `sidewalk_sdk_qa` - special build of Sidewalk SDK specifically for the `sid_dut` app. Enables additional QA features that are not required in normal applications
- `sidewalk_halo_diagnostics` - Amazon's proprietary application code for `sid_diagnostics` application
- `sidewalk_sdk_diagnostics` - special build of Sidewalk SDK specifically for the `sid_diagnostics` app. Enables additional PHY-level diagnostic features that are not required in normal applications

# Basic Sidewalk Stack Limitations

The following features were excluded from the basic variant of the Sidewalk stack to minimize flash and RAM space consumption:
- Autoconnect
- Multilink mode
- Sidewalk Bulk Data Transfer (SBDT)

If your application requires any of the above features please use the full variant of the stack.

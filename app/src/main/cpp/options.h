// Copyright (c) 2017-2020 The Khronos Group Inc
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

struct Options {
    std::string GraphicsPlugin;

    std::string FormFactor{"Hmd"};

    std::string ViewConfiguration{"Stereo"};

    std::string EnvironmentBlendMode{"Opaque"};

    std::string AppSpace{"Stage"};  // ★ 修改为 Stage，这样 Y=0 就在真实的物理地板上，而不是在头部的视线高度上

    struct {
        XrFormFactor FormFactor{XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};

        XrViewConfigurationType ViewConfigType{XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};

        XrEnvironmentBlendMode EnvironmentBlendMode{XR_ENVIRONMENT_BLEND_MODE_OPAQUE};
    } Parsed;
};

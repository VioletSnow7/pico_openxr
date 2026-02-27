// Copyright (c) 2017-2022, The Khronos Group Inc.
//
// SPDX-License-Identifier: Apache-2.0

// ============================================================================
// main.cpp - 程序入口文件
// ============================================================================
// 这是整个 OpenXR Demos 程序的入口点。
// 由于 MainActivity 继承自 NativeActivity，Android 会在原生线程中调用 android_main()，
// 而不是在 Java 线程中运行。
//
// 主要职责：
// 1. 初始化 JNI 环境和 Android 事件处理
// 2. 读取系统属性确定使用的图形 API (OpenGL ES / Vulkan)
// 3. 创建平台插件、图形插件和 OpenXR 程序三大核心对象
// 4. 初始化 OpenXR Loader 和整个渲染管线
// 5. 运行主循环：事件处理 → 输入轮询 → 帧渲染
// ============================================================================

#include "pch.h"           // 预编译头文件（标准库 + 平台头文件 + 图形API + OpenXR）
#include "common.h"        // 公共工具函数和宏（字符串比较、枚举转字符串等）
#include "options.h"       // 运行时选项配置（图形插件类型、视图配置等）
#include "platformdata.h"  // 平台特定数据（JVM 和 Activity 引用）
#include "platformplugin.h"  // 平台插件接口（提供 Android 平台特定的 OpenXR 扩展）
#include "graphicsplugin.h"  // 图形插件接口（封装 OpenGL ES / Vulkan 图形操作）
#include "openxr_program.h"  // OpenXR 主程序接口（核心 XR 生命周期管理）
#include "demos/utils.h"     // 工具函数（纹理加载、文件操作等）

namespace {

// 显示帮助信息：可以通过 adb shell setprop 设置运行时参数
void ShowHelp() {
    Log::Write(Log::Level::Info, "adb shell setprop debug.xr.graphicsPlugin OpenGLES|Vulkan");
    Log::Write(Log::Level::Info, "adb shell setprop debug.xr.formFactor Hmd|Handheld");
    Log::Write(Log::Level::Info, "adb shell setprop debug.xr.viewConfiguration Stereo|Mono");
    Log::Write(Log::Level::Info, "adb shell setprop debug.xr.blendMode Opaque|Additive|AlphaBlend");
    Log::Write(Log::Level::Info, "adb shell setprop persist.log.tag V");
}

// 从 Android 系统属性中读取运行时配置
// 可以通过 "adb shell setprop debug.xr.graphicsPlugin OpenGLES" 来指定图形API
// 如果未设置，默认使用 OpenGL ES
bool UpdateOptionsFromSystemProperties(Options& options) {
    char value[PROP_VALUE_MAX] = {};
    // 从系统属性读取图形插件类型
    if (__system_property_get("debug.xr.graphicsPlugin", value) != 0) {
        options.GraphicsPlugin = value;
    }

    // 如果未指定图形插件，默认使用 OpenGL ES
    if (options.GraphicsPlugin.empty()) {
        Log::Write(Log::Level::Warning, __FILE__, __LINE__, "GraphicsPlugin Default OpenGLES");
        options.GraphicsPlugin = "OpenGLES";
    }
    return true;
}
}  // namespace


// Android 应用状态结构体
// 用于在事件回调和主循环之间共享状态信息
struct AndroidAppState {
    ANativeWindow* NativeWindow = nullptr;  // 原生窗口句柄（Surface 创建/销毁时更新）
    bool Resumed = false;                   // 应用是否处于前台运行状态
    std::shared_ptr<IOpenXrProgram> program; // OpenXR 主程序实例
};

/**
 * 处理 Android 应用生命周期命令的回调函数
 * 当 Android 系统发送生命周期事件（如暂停、恢复、窗口创建/销毁等）时被调用
 * 注意：没有 APP_CMD_CREATE，因为 NativeActivity 在 onCreate() 中
 * 创建了应用线程，线程随后调用 android_main()
 */
static void app_handle_cmd(struct android_app* app, int32_t cmd) {
    AndroidAppState* appState = (AndroidAppState*)app->userData;

    switch (cmd) {
        case APP_CMD_START: {
            // 应用启动 → 对应 Activity.onStart()
            Log::Write(Log::Level::Info, __FILE__, __LINE__, "onStart()");
            break;
        }
        case APP_CMD_RESUME: {
            // 应用恢复到前台 → 对应 Activity.onResume()
            Log::Write(Log::Level::Info, __FILE__, __LINE__, "onResume()");
            appState->Resumed = true;
            if (appState->program.get()) {
                // 可在此处恢复 XR 会话相关操作
            }
            break;
        }
        case APP_CMD_PAUSE: {
            // 应用进入后台 → 对应 Activity.onPause()
            Log::Write(Log::Level::Info, __FILE__, __LINE__, "onPause()");
            appState->Resumed = false;
            if (appState->program.get()) {
                // 可在此处暂停 XR 会话相关操作
            }
            break;
        }
        case APP_CMD_STOP: {
            // 应用停止 → 对应 Activity.onStop()
            Log::Write(Log::Level::Info, __FILE__, __LINE__, "onStop()");
            break;
        }
        case APP_CMD_DESTROY: {
            // 应用销毁 → 对应 Activity.onDestroy()
            Log::Write(Log::Level::Info, __FILE__, __LINE__, "onDestroy()");
            appState->NativeWindow = NULL;
            break;
        }
        case APP_CMD_INIT_WINDOW: {
            // Surface 创建完成，可以开始渲染
            Log::Write(Log::Level::Info, __FILE__, __LINE__, "surfaceCreated()");
            appState->NativeWindow = app->window;
            break;
        }
        case APP_CMD_TERM_WINDOW: {
            // Surface 销毁，停止渲染
            Log::Write(Log::Level::Info, __FILE__, __LINE__, "surfaceDestroyed()");
            appState->NativeWindow = NULL;
            break;
        }
    }
}

// Android 原生输入事件回调
// 处理物理按键事件（如音量键），返回 0 表示不消费该事件
static int32_t onInputEvent(struct android_app* app, AInputEvent* event){
    int type = AInputEvent_getType(event);
    if(type == AINPUT_EVENT_TYPE_KEY){
        int32_t action = AKeyEvent_getAction(event);  // 按下/抬起
        int32_t code   = AKeyEvent_getKeyCode(event); // 按键码
        Log::Write(Log::Level::Info, __FILE__, __LINE__, Fmt("onInputEvent:%d %d\n", code, action));
    }
    return 0;
}

/**
 * 程序主入口函数 (Native Application 入口)
 * 
 * 使用 android_native_app_glue 框架，运行在独立的原生线程中，
 * 拥有自己的事件循环来接收输入事件等。
 *
 * 完整的执行流程：
 * 1. JNI 线程绑定
 * 2. 读取系统属性配置
 * 3. 创建三大核心对象（平台插件、图形插件、OpenXR程序）
 * 4. 初始化 OpenXR Loader
 * 5. 5步初始化：CreateInstance → InitializeSystem → InitializeSession → CreateSwapchains → InitializeApplication
 * 6. 进入主循环：事件处理 → 输入轮询 → 帧渲染
 */
void android_main(struct android_app* app) {
    try {
        // ========== 第1步：JNI 环境初始化 ==========
        JNIEnv* Env;
        // 将当前原生线程绑定到 Java VM，以便后续可以调用 Java 方法
        app->activity->vm->AttachCurrentThread(&Env, nullptr);
        // 保存 JNI 环境指针，供 utils.cpp 等模块使用（如访问 Assets、媒体扫描等）
        setJNIEnv(Env);

        // ========== 第2步：注册 Android 事件回调 ==========
        AndroidAppState appState = {};
        app->userData = &appState;           // 将应用状态传给回调函数
        app->onAppCmd = app_handle_cmd;      // 注册生命周期事件回调
        app->onInputEvent = onInputEvent;    // 注册输入事件回调

        // ========== 第3步：读取运行时配置 ==========
        std::shared_ptr<Options> options = std::make_shared<Options>();
        if (!UpdateOptionsFromSystemProperties(*options)) {
            return;  // 配置读取失败则退出
        }

        // ========== 第4步：准备平台数据 ==========
        std::shared_ptr<PlatformData> data = std::make_shared<PlatformData>();
        data->applicationVM = app->activity->vm;           // Java 虚拟机指针
        data->applicationActivity = app->activity->clazz;  // Activity 实例引用

        bool requestRestart = false;  // 是否请求重启会话
        bool exitRenderLoop = false;  // 是否退出渲染循环

        // ========== 第5步：创建三大核心对象 ==========
        // 创建平台插件 → 提供 Android 平台特有的 OpenXR 实例创建扩展
        std::shared_ptr<IPlatformPlugin> platformPlugin = CreatePlatformPlugin(options, data);
        // 创建图形插件 → 封装 OpenGL ES 或 Vulkan 的初始化和渲染操作
        std::shared_ptr<IGraphicsPlugin> graphicsPlugin = CreateGraphicsPlugin(options, platformPlugin);
        // 创建 OpenXR 主程序 → 管理整个 XR 生命周期（会话、输入、渲染）
        std::shared_ptr<IOpenXrProgram> program = CreateOpenXrProgram(options, platformPlugin, graphicsPlugin);

        appState.program = program;

        // ========== 第6步：初始化 OpenXR Loader ==========
        // Loader 是 OpenXR 运行时的入口点，负责发现和加载设备上的 XR 运行时
        // 在 PICO 设备上，运行时由 PICO 的系统服务提供
        PFN_xrInitializeLoaderKHR initializeLoader = nullptr;
        if (XR_SUCCEEDED(xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR", (PFN_xrVoidFunction*)(&initializeLoader)))) {
            XrLoaderInitInfoAndroidKHR loaderInitInfoAndroid;
            memset(&loaderInitInfoAndroid, 0, sizeof(loaderInitInfoAndroid));
            loaderInitInfoAndroid.type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR;
            loaderInitInfoAndroid.next = NULL;
            loaderInitInfoAndroid.applicationVM = app->activity->vm;       // 传入 JVM
            loaderInitInfoAndroid.applicationContext = app->activity->clazz; // 传入 Activity
            initializeLoader((const XrLoaderInitInfoBaseHeaderKHR*)&loaderInitInfoAndroid);
        }

        // ========== 第7步：5步初始化 OpenXR ==========
        program->CreateInstance();           // 1. 创建 XrInstance（注册扩展、日志层）
        program->InitializeSystem();         // 2. 获取 XrSystemId，初始化图形设备
        program->InitializeSession();        // 3. 创建 XrSession，初始化输入/追踪
        program->CreateSwapchains();         // 4. 创建左右眼渲染交换链
        program->InitializeApplication();    // 5. 初始化应用层（手柄、GUI、播放器等）

        // ========== 第8步：主循环 ==========
        // 持续运行直到应用被销毁
        while (app->destroyRequested == 0) {
            // --- 8.1 处理所有待处理的 Android 事件 ---
            for (;;) {
                int events;
                struct android_poll_source* source;
                // 如果应用未激活且 XR 会话未运行，则阻塞等待事件（省电）
                // 否则立即返回（保证渲染帧率）
                const int timeoutMilliseconds = (!appState.Resumed && !program->IsSessionRunning() && app->destroyRequested == 0) ? -1 : 0;
                if (ALooper_pollAll(timeoutMilliseconds, nullptr, &events, (void**)&source) < 0) {
                    break;  // 没有更多事件，退出事件处理循环
                }
                // 分发处理事件
                if (source != nullptr) {
                    source->process(app, source);
                }
            }

            // --- 8.2 处理 OpenXR 事件 ---
            // 包括会话状态变化（READY/STOPPING/LOSS_PENDING等）
            program->PollEvents(&exitRenderLoop, &requestRestart);

            // 如果需要退出且不需要重启，则结束 Activity
            if (exitRenderLoop && !requestRestart) {
                ANativeActivity_finish(app->activity);
            }

            // 如果 XR 会话未运行，降低循环频率（因为不会调用 xrWaitFrame）
            if (!program->IsSessionRunning()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
                continue;
            }

            // --- 8.3 轮询输入动作 ---
            // 读取手柄按键、摇杆、扳机，以及眼动追踪、手部追踪数据
            program->PollActions();

            // --- 8.4 渲染帧 ---
            // xrWaitFrame → xrBeginFrame → 渲染左右眼 → xrEndFrame
            program->RenderFrame();
        }

        // 分离 JNI 线程
        app->activity->vm->DetachCurrentThread();
    }
    catch (const std::exception &ex)
    {
        // 捕获已知异常并输出日志
        Log::Write(Log::Level::Error, __FILE__, __LINE__, ex.what());
    }
    catch (...)
    {
        // 捕获未知异常
        Log::Write(Log::Level::Error, __FILE__, __LINE__, "Unknown Error");
    }
}

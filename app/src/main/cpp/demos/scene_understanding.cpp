/* 
 * 场景理解模块实现 - Scene Understanding Implementation
 * 
 * 使用 XR_FB_scene 系列扩展实现真实世界平面检测
 */
#include "scene_understanding.h"
#include <cstring>
#include <algorithm>
#include <cmath>

SceneUnderstanding::SceneUnderstanding() {}
SceneUnderstanding::~SceneUnderstanding() {}

// ====================================================================
// 获取需要注册的 OpenXR 扩展列表
// 在 CreateInstanceInternal() 中调用
// ====================================================================
std::vector<const char*> SceneUnderstanding::GetRequiredExtensions() {
    return {
        XR_FB_SPATIAL_ENTITY_EXTENSION_NAME,           // 空间实体基础
        XR_FB_SPATIAL_ENTITY_QUERY_EXTENSION_NAME,     // 查询空间锚点
        XR_FB_SPATIAL_ENTITY_CONTAINER_EXTENSION_NAME, // 房间容器
        XR_FB_SCENE_EXTENSION_NAME,                     // 场景语义（桌面、地面等）
        XR_FB_SCENE_CAPTURE_EXTENSION_NAME,             // 触发房间扫描
    };
}

// ====================================================================
// 初始化函数指针（在 xrCreateInstance 之后调用）
// ====================================================================
bool SceneUnderstanding::initializeFunctionPointers(XrInstance instance) {
    mInstance = instance;

    // XR_FB_scene
    xrGetInstanceProcAddr(instance, "xrGetSpaceSemanticLabelsFB",
        (PFN_xrVoidFunction*)&pfnGetSpaceSemanticLabelsFB);
    xrGetInstanceProcAddr(instance, "xrGetSpaceBoundingBox2DFB",
        (PFN_xrVoidFunction*)&pfnGetSpaceBoundingBox2DFB);
    xrGetInstanceProcAddr(instance, "xrGetSpaceBoundingBox3DFB",
        (PFN_xrVoidFunction*)&pfnGetSpaceBoundingBox3DFB);
    xrGetInstanceProcAddr(instance, "xrGetSpaceRoomLayoutFB",
        (PFN_xrVoidFunction*)&pfnGetSpaceRoomLayoutFB);
    xrGetInstanceProcAddr(instance, "xrGetSpaceBoundary2DFB",
        (PFN_xrVoidFunction*)&pfnGetSpaceBoundary2DFB);

    // XR_FB_scene_capture
    xrGetInstanceProcAddr(instance, "xrRequestSceneCaptureFB",
        (PFN_xrVoidFunction*)&pfnRequestSceneCaptureFB);

    // XR_FB_spatial_entity_query
    xrGetInstanceProcAddr(instance, "xrQuerySpacesFB",
        (PFN_xrVoidFunction*)&pfnQuerySpacesFB);
    xrGetInstanceProcAddr(instance, "xrRetrieveSpaceQueryResultsFB",
        (PFN_xrVoidFunction*)&pfnRetrieveSpaceQueryResultsFB);

    // XR_FB_spatial_entity
    xrGetInstanceProcAddr(instance, "xrEnumerateSpaceSupportedComponentsFB",
        (PFN_xrVoidFunction*)&pfnEnumerateSpaceSupportedComponentsFB);
    xrGetInstanceProcAddr(instance, "xrSetSpaceComponentStatusFB",
        (PFN_xrVoidFunction*)&pfnSetSpaceComponentStatusFB);
    xrGetInstanceProcAddr(instance, "xrGetSpaceComponentStatusFB",
        (PFN_xrVoidFunction*)&pfnGetSpaceComponentStatusFB);

    // XR_FB_spatial_entity_container
    xrGetInstanceProcAddr(instance, "xrGetSpaceContainerFB",
        (PFN_xrVoidFunction*)&pfnGetSpaceContainerFB);

    // 检查关键函数是否可用
    bool allAvailable = (pfnGetSpaceSemanticLabelsFB != nullptr)
        && (pfnQuerySpacesFB != nullptr)
        && (pfnRetrieveSpaceQueryResultsFB != nullptr)
        && (pfnRequestSceneCaptureFB != nullptr);

    if (!allAvailable) {
        Log::Write(Log::Level::Warning, "SceneUnderstanding: Some XR_FB_scene functions not available. Device may not support scene understanding.");
    } else {
        Log::Write(Log::Level::Info, "SceneUnderstanding: All function pointers initialized successfully.");
    }

    return allAvailable;
}

// ====================================================================
// 初始化场景理解（在 session 创建后调用）
// ====================================================================
bool SceneUnderstanding::initialize(XrSession session, XrSpace appSpace) {
    mSession = session;
    mAppSpace = appSpace;
    Log::Write(Log::Level::Info, "SceneUnderstanding: Initialized with session and app space.");
    return true;
}

// ====================================================================
// 请求房间扫描（会打开系统的房间扫描界面）
// 用户需要在系统界面中完成房间扫描
// ====================================================================
bool SceneUnderstanding::requestSceneCapture() {
    if (!pfnRequestSceneCaptureFB || mSession == XR_NULL_HANDLE) {
        Log::Write(Log::Level::Error, "SceneUnderstanding: Cannot request scene capture - not initialized.");
        return false;
    }

    // 构造扫描请求
    XrSceneCaptureRequestInfoFB captureInfo{XR_TYPE_SCENE_CAPTURE_REQUEST_INFO_FB};
    captureInfo.requestByteCount = 0;
    captureInfo.request = nullptr;

    // 发送异步请求
    XrResult result = pfnRequestSceneCaptureFB(mSession, &captureInfo, &mCaptureRequestId);
    if (result != XR_SUCCESS) {
        Log::Write(Log::Level::Error, Fmt("SceneUnderstanding: xrRequestSceneCaptureFB failed: %d", result));
        return false;
    }

    mWaitingForCapture = true;
    Log::Write(Log::Level::Info, Fmt("SceneUnderstanding: Scene capture requested, requestId: %lld", mCaptureRequestId));
    return true;
}

// ====================================================================
// 查询已有的场景数据（不需要重新扫描）
// 如果用户之前已经扫描过房间，可以直接查询数据
// ====================================================================
bool SceneUnderstanding::querySceneAnchors() {
    if (!pfnQuerySpacesFB || mSession == XR_NULL_HANDLE) {
        Log::Write(Log::Level::Error, "SceneUnderstanding: Cannot query scene - not initialized.");
        return false;
    }

    // 构造查询条件：查找所有包含语义标签的空间实体
    XrSpaceComponentFilterInfoFB componentFilter{XR_TYPE_SPACE_COMPONENT_FILTER_INFO_FB};
    componentFilter.componentType = XR_SPACE_COMPONENT_TYPE_SEMANTIC_LABELS_FB;

    // 存储位置过滤器
    XrSpaceStorageLocationFilterInfoFB storageFilter{XR_TYPE_SPACE_STORAGE_LOCATION_FILTER_INFO_FB};
    storageFilter.location = XR_SPACE_STORAGE_LOCATION_LOCAL_FB;

    XrSpaceQueryInfoFB queryInfo{XR_TYPE_SPACE_QUERY_INFO_FB};
    queryInfo.queryAction = XR_SPACE_QUERY_ACTION_LOAD_FB;
    queryInfo.maxResultCount = 100;   // 最多查询 100 个场景锚点
    queryInfo.timeout = XR_INFINITE_DURATION;
    queryInfo.filter = (const XrSpaceFilterInfoBaseHeaderFB*)&componentFilter;
    queryInfo.excludeFilter = nullptr;

    // 将 storageFilter 链接到 componentFilter 的 next 中
    componentFilter.next = &storageFilter;

    // 发送异步查询请求
    XrResult result = pfnQuerySpacesFB(
        mSession,
        (const XrSpaceQueryInfoBaseHeaderFB*)&queryInfo,
        &mQueryRequestId);

    if (result != XR_SUCCESS) {
        Log::Write(Log::Level::Error, Fmt("SceneUnderstanding: xrQuerySpacesFB failed: %d", result));
        return false;
    }

    mWaitingForQuery = true;
    Log::Write(Log::Level::Info, Fmt("SceneUnderstanding: Scene query sent, requestId: %lld", mQueryRequestId));
    return true;
}

// ====================================================================
// 处理异步事件回调
// 需要在 PollEvents 中调用
// ====================================================================
void SceneUnderstanding::handleEvent(const XrEventDataBaseHeader* event) {
    // 处理场景扫描完成事件
    if (event->type == XR_TYPE_EVENT_DATA_SCENE_CAPTURE_COMPLETE_FB) {
        const auto* captureComplete = reinterpret_cast<const XrEventDataSceneCaptureCompleteFB*>(event);
        Log::Write(Log::Level::Info, Fmt("SceneUnderstanding: Scene capture complete, result: %d", captureComplete->result));

        mWaitingForCapture = false;

        if (captureComplete->result == XR_SUCCESS) {
            // 扫描成功，立即查询场景数据
            Log::Write(Log::Level::Info, "SceneUnderstanding: Scene capture successful, querying scene data...");
            querySceneAnchors();
        }
        return;
    }

    // 处理查询结果可用事件
    if (event->type == XR_TYPE_EVENT_DATA_SPACE_QUERY_RESULTS_AVAILABLE_FB) {
        const auto* queryAvailable = reinterpret_cast<const XrEventDataSpaceQueryResultsAvailableFB*>(event);
        Log::Write(Log::Level::Info, Fmt("SceneUnderstanding: Query results available, requestId: %lld", queryAvailable->requestId));

        if (queryAvailable->requestId == mQueryRequestId) {
            processQueryResults();
        }
        return;
    }

    // 处理查询完成事件
    if (event->type == XR_TYPE_EVENT_DATA_SPACE_QUERY_COMPLETE_FB) {
        const auto* queryComplete = reinterpret_cast<const XrEventDataSpaceQueryCompleteFB*>(event);
        Log::Write(Log::Level::Info, Fmt("SceneUnderstanding: Query complete, result: %d, planes found: %d",
            queryComplete->result, (int)mPlanes.size()));

        mWaitingForQuery = false;
        mSceneDataAvailable = !mPlanes.empty();
        return;
    }
}

// ====================================================================
// 处理查询结果
// ====================================================================
void SceneUnderstanding::processQueryResults() {
    if (!pfnRetrieveSpaceQueryResultsFB) return;

    // 先获取结果数量
    XrSpaceQueryResultsFB queryResults{XR_TYPE_SPACE_QUERY_RESULTS_FB};
    queryResults.resultCapacityInput = 0;
    queryResults.resultCountOutput = 0;
    queryResults.results = nullptr;

    pfnRetrieveSpaceQueryResultsFB(mSession, mQueryRequestId, &queryResults);

    if (queryResults.resultCountOutput == 0) {
        Log::Write(Log::Level::Info, "SceneUnderstanding: No query results.");
        return;
    }

    // 分配内存并获取结果
    std::vector<XrSpaceQueryResultFB> results(queryResults.resultCountOutput);
    queryResults.resultCapacityInput = queryResults.resultCountOutput;
    queryResults.results = results.data();

    XrResult res = pfnRetrieveSpaceQueryResultsFB(mSession, mQueryRequestId, &queryResults);
    if (res != XR_SUCCESS) {
        Log::Write(Log::Level::Error, Fmt("SceneUnderstanding: Failed to retrieve query results: %d", res));
        return;
    }

    Log::Write(Log::Level::Info, Fmt("SceneUnderstanding: Processing %d scene anchors...", queryResults.resultCountOutput));

    // 处理每个场景锚点
    for (uint32_t i = 0; i < queryResults.resultCountOutput; i++) {
        XrSpace space = results[i].space;
        XrUuidEXT uuid = results[i].uuid;

        // 启用 LOCATABLE 组件（用于定位平面位置）
        if (pfnSetSpaceComponentStatusFB) {
            XrSpaceComponentStatusSetInfoFB setInfo{XR_TYPE_SPACE_COMPONENT_STATUS_SET_INFO_FB};
            setInfo.componentType = XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB;
            setInfo.enabled = XR_TRUE;
            setInfo.timeout = XR_INFINITE_DURATION;
            XrAsyncRequestIdFB reqId;
            pfnSetSpaceComponentStatusFB(space, &setInfo, &reqId);
        }

        // 获取语义标签
        std::string label = getSemanticLabel(space);
        if (label.empty()) continue;

        // 获取 2D 边界框
        float width = 0, height = 0;
        getBoundingBox2D(space, width, height);

        // 创建平面信息
        DetectedPlane plane;
        plane.label = label;
        plane.space = space;
        plane.uuid = uuid;
        plane.width = width;
        plane.height = height;
        plane.pose = {{0, 0, 0, 1}, {0, 0, 0}};
        plane.surfaceY = 0;

        // 判断是否为水平面
        plane.isHorizontal = (label == "TABLE" || label == "DESK" ||
                              label == "FLOOR" || label == "CEILING" ||
                              label == "COUCH" || label == "BED" ||
                              label == "OTHER");

        mPlanes.push_back(plane);

        Log::Write(Log::Level::Info, Fmt("SceneUnderstanding: Found plane [%s] size: %.2f x %.2f",
            label.c_str(), width, height));
    }

    Log::Write(Log::Level::Info, Fmt("SceneUnderstanding: Total %d planes detected.", (int)mPlanes.size()));
}

// ====================================================================
// 获取语义标签
// ====================================================================
std::string SceneUnderstanding::getSemanticLabel(XrSpace space) {
    if (!pfnGetSpaceSemanticLabelsFB) return "";

    // 先获取标签长度
    XrSemanticLabelsFB labels{XR_TYPE_SEMANTIC_LABELS_FB};

    // 添加语义标签支持信息
    XrSemanticLabelsSupportInfoFB supportInfo{XR_TYPE_SEMANTIC_LABELS_SUPPORT_INFO_FB};
    supportInfo.flags = XR_SEMANTIC_LABELS_SUPPORT_ACCEPT_DESK_TO_TABLE_MIGRATION_BIT_FB;
    supportInfo.recognizedLabels = "TABLE,FLOOR,CEILING,WALL,DOOR,WINDOW,COUCH,DESK,BED,LAMP,SCREEN,OTHER";
    labels.next = &supportInfo;

    labels.bufferCapacityInput = 0;
    XrResult res = pfnGetSpaceSemanticLabelsFB(mSession, space, &labels);
    if (res != XR_SUCCESS || labels.bufferCountOutput == 0) {
        return "";
    }

    // 分配缓冲区并获取标签
    std::vector<char> buffer(labels.bufferCountOutput);
    labels.bufferCapacityInput = labels.bufferCountOutput;
    labels.buffer = buffer.data();

    res = pfnGetSpaceSemanticLabelsFB(mSession, space, &labels);
    if (res != XR_SUCCESS) {
        return "";
    }

    // 标签可能包含多个，用逗号分隔，取第一个
    std::string fullLabel(buffer.data(), labels.bufferCountOutput - 1); // 去掉末尾null
    size_t commaPos = fullLabel.find(',');
    if (commaPos != std::string::npos) {
        return fullLabel.substr(0, commaPos);
    }
    return fullLabel;
}

// ====================================================================
// 获取 2D 边界框
// ====================================================================
bool SceneUnderstanding::getBoundingBox2D(XrSpace space, float& width, float& height) {
    if (!pfnGetSpaceBoundingBox2DFB) return false;

    XrRect2Df boundingBox;
    XrResult res = pfnGetSpaceBoundingBox2DFB(mSession, space, &boundingBox);
    if (res != XR_SUCCESS) {
        return false;
    }

    width = boundingBox.extent.width;
    height = boundingBox.extent.height;
    return true;
}

// ====================================================================
// 更新平面的空间位置（每帧调用）
// ====================================================================
void SceneUnderstanding::updatePlaneLocations(XrTime predictedDisplayTime) {
    if (!mSceneDataAvailable || mAppSpace == XR_NULL_HANDLE) return;

    for (auto& plane : mPlanes) {
        XrSpaceLocation spaceLocation{XR_TYPE_SPACE_LOCATION};
        XrResult res = xrLocateSpace(plane.space, mAppSpace, predictedDisplayTime, &spaceLocation);
        if (res == XR_SUCCESS) {
            if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
                (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                plane.pose = spaceLocation.pose;
                // 对于水平面，surfaceY 就是平面的 Y 坐标
                if (plane.isHorizontal) {
                    plane.surfaceY = spaceLocation.pose.position.y;
                }
            }
        }
    }
}

// ====================================================================
// 获取最近的水平面高度（用于碰撞检测）
// 在给定 (x, y, z) 位置，找到下方最近的水平面的 Y 坐标
// ====================================================================
float SceneUnderstanding::getNearestSurfaceY(float x, float y, float z) const {
    float bestY = 0.0f; // 默认地面
    float minDistance = 999.0f;

    for (const auto& plane : mPlanes) {
        if (!plane.isHorizontal) continue;

        // 只考虑物体下方的平面
        if (plane.surfaceY > y) continue;

        // 检查 (x, z) 是否在平面范围内（简化判断：使用距离）
        float dx = x - plane.pose.position.x;
        float dz = z - plane.pose.position.z;
        float horizontalDist = std::sqrt(dx * dx + dz * dz);

        // 如果水平距离在合理范围内（平面大小的一半 + 缓冲区）
        float planeRadius = std::max(plane.width, plane.height) / 2.0f + 0.5f;
        if (horizontalDist < planeRadius) {
            float vertDist = y - plane.surfaceY;
            if (vertDist < minDistance) {
                minDistance = vertDist;
                bestY = plane.surfaceY;
            }
        }
    }

    return bestY;
}

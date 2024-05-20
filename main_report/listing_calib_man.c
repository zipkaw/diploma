static inline bool isValidAccelerometerCalibrationState(
    businessLayer_calibrationManagerState_t current_state, businessLayer_calibrationManagerState_t requested_state)
{
    return current_state == requested_state && (
        requested_state == BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_Z_UP ||
        requested_state == BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_Z_DOWN ||
        requested_state == BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_Y_UP ||
        requested_state == BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_Y_DOWN ||
        requested_state == BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_X_UP ||
        requested_state == BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_X_DOWN);
}
static inline businessLayer_calibrationManagerReturnCode_t convertMagnetometerToManagerCode(
    businessLayer_calibrationMagnetometerReturnCode_t code)
{
    return code == BUSINESS_LAYER_CALIBRATION_MANAGER_OK
               ? BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_OK
               : BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INTERNAL_ERROR;
}
static inline businessLayer_calibrationManagerReturnCode_t convertAccelerometerToManagerCode(
    businessLayer_calibrationAccelerometerReturnCode_t code)
{
    return code == BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK
               ? BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_OK
               : BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INTERNAL_ERROR;
}
static inline businessLayer_calibrationManagerReturnCode_t convertGyroscopeToManagerCode(
    businessLayer_calibrationGyroscopeReturnCode_t code)
{
    return code == BUSINESS_LAYER_CALIBRATION_GYROSCOPE_OK
               ? BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_OK
               : BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INTERNAL_ERROR;
}
static void accelerometerCancelCallback(
    void *calibration_manager_handle, businessLayer_calibrationAccelerometerReturnCode_t code)
{
    SANITY_CHECK_C(calibration_manager_handle != NULL, SANITY_CHECK_RET_NOTHING, "Calibration manager handle is NULL");

    businessLayer_calibrationManager_t handle = calibration_manager_handle;

    handle->cancelCode = convertAccelerometerToManagerCode(code);
    handle->wasCancelCallback = true;
}
static businessLayer_calibrationManagerReturnCode_t 
cancelCurrentCalibration(businessLayer_calibrationManager_t handle)
{
    SANITY_CHECK_C(handle != BUSINESS_LAYER_CALIBRATION_MANAGER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "Calibration manager handle is invalid");

    switch (handle->calibrationStage.target)
    {
        case GENERAL_CALIBRATION_TARGET_MAGNETOMETER:
        {
            const businessLayer_calibrationMagnetometerReturnCode_t code =
                businessLayer_calibrationMagnetometerCancelRecalibrationAsync(
                    handle->handles.magnetometer, magnetometerCancelCallback, handle);

            return convertMagnetometerToManagerCode(code);
        }
        case GENERAL_CALIBRATION_TARGET_ACCELEROMETER:
        {
            const businessLayer_calibrationAccelerometerReturnCode_t code =
                businessLayer_calibrationAccelerometerCancelRecalibrateAsync(
                    handle->handles.accelerometer, accelerometerCancelCallback, handle);

            return convertAccelerometerToManagerCode(code);
        }
        case GENERAL_CALIBRATION_TARGET_GYROSCOPE:
        {
            const businessLayer_calibrationGyroscopeReturnCode_t code =
                businessLayer_calibrationGyroscopeCancelRecalibrateAsync(
                    handle->handles.gyroscope, gyroscopeCancelCallback, handle);

            return convertGyroscopeToManagerCode(code);
        }
        default:
        {
            osal_logMsgE("Unknown request calibration target: %" PRIu8, handle->calibrationStage.target);
            return BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT;
        }
    }
}
static void accelerometerCalibrationCallback(void *calibration_manager_handle,
    businessLayer_calibrationAccelerometerReturnCode_t code, businessLayer_calibrationAccelerometerStates_t state)
{
    SANITY_CHECK_C(calibration_manager_handle != NULL, SANITY_CHECK_RET_NOTHING, "Calibration manager handle is NULL");

    businessLayer_calibrationManager_t handle = (businessLayer_calibrationManager_t)calibration_manager_handle;

    if (code == BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_CANCEL)
    {
        // Don't care
        return;
    }

    SANITY_CHECK_C(handle->calibrationStage.target == GENERAL_CALIBRATION_TARGET_ACCELEROMETER,
        SANITY_CHECK_RET_NOTHING,
        "Accelerometer calibration callback called when internal target is not accelerometer. Internal target: %" PRIu8,
        handle->calibrationStage.target);

    bool is_response_required = true;
    switch (state)
    {
        case BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_POSITION_Z_DOWN:
        {
            handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_Z_DOWN;
            break;
        }
        case BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_POSITION_Y_UP:
        {
            handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_Y_UP;
            break;
        }
        case BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_POSITION_Y_DOWN:
        {
            handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_Y_DOWN;
            break;
        }
        case BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_POSITION_X_UP:
        {
            handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_X_UP;
            break;
        }
        case BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_POSITION_X_DOWN:
        {
            handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_X_DOWN;
            break;
        }
        case BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_POSITION_FINISH:
        {
            is_response_required = false;
            break;
        }
        case BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_CALIBRATION_WAS_FINISHED:
        {
            handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_FINISH;
            break;
        }
        default:
        {
            osal_logMsgC("Unexpected internal accelerometer calibration state: %i", state);
            handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_ERROR;
            break;
        }
    }
    if (code != BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK)
    {
        osal_logMsgE("Accelerometer calibration error in callback: %i", code);
        handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_ERROR;
        is_response_required = true;
    }
    if (is_response_required)
    {
        SANITY_CHECK_C(handle->userCallback != NULL, SANITY_CHECK_RET_NOTHING, "User callback is NULL");
        handle->userCallback(handle->userCallbackArg, handle->calibrationStage);

        handle->userCallback = NULL;
        handle->userCallbackArg = NULL;

        if (handle->calibrationStage.state == BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_ERROR)
        {
            handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_FINISH;
        }
    }
}
static businessLayer_calibrationManagerReturnCode_t serveAccelerometerCalibration(
    businessLayer_calibrationManager_t handle, businessLayer_calibrationManagerState_t state)
{
    SANITY_CHECK_C(handle != BUSINESS_LAYER_CALIBRATION_MANAGER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "Invalid calibration manager handle");

    if (state == BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_START)
    {
        SANITY_CHECK_ALWAYS_E(handle->calibrationStage.state == BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_FINISH,
            BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_STATE,
            "Request for START is valid only when internal state is %i. Current internal state: %" PRIu8,
            BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_FINISH, handle->calibrationStage.state);

        const businessLayer_calibrationAccelerometerReturnCode_t code =
            businessLayer_calibrationAccelerometerRecalibrateAsync(
                handle->handles.accelerometer, accelerometerCalibrationCallback, handle);
        SANITY_CHECK_ALWAYS_E(code == BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK,
            BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INTERNAL_ERROR,
            "Failed to start accelerometer calibration: %i", code);
        handle->calibrationStage.target = GENERAL_CALIBRATION_TARGET_ACCELEROMETER;
        handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_Z_UP;

        SANITY_CHECK_C(handle->userCallback != NULL, 
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INTERNAL_ERROR,
            "User callback is NULL");
        handle->userCallback(handle->userCallbackArg, handle->calibrationStage);

        handle->userCallback = NULL;
        handle->userCallbackArg = NULL;

        return BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_OK;
    }

    SANITY_CHECK_ALWAYS_E(isValidAccelerometerCalibrationState(handle->calibrationStage.state, state),
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_STATE, "Accelerometer calibration unexpected state: %i",
        state);

    const businessLayer_calibrationAccelerometerReturnCode_t code =
        businessLayer_calibrationAccelerometerPositionWasSetCb(handle->handles.accelerometer);
    SANITY_CHECK_ALWAYS_E(code == BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INTERNAL_ERROR,
        "Failed to set new accelerometer calibration position. Request state: %i", state);

    return BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_OK;
}
static void gyroscopeCalibrationCallback(
    void *calibration_manager_handle, businessLayer_calibrationGyroscopeReturnCode_t code)
{
    SANITY_CHECK_C(calibration_manager_handle != NULL, SANITY_CHECK_RET_NOTHING, "Calibration manager handle is NULL");

    businessLayer_calibrationManager_t handle = (businessLayer_calibrationManager_t)calibration_manager_handle;

    if (code == BUSINESS_LAYER_CALIBRATION_GYROSCOPE_CANCEL)
    {
        // Don't care
        return;
    }

    SANITY_CHECK_C(handle->calibrationStage.target == 
    GENERAL_CALIBRATION_TARGET_GYROSCOPE, SANITY_CHECK_RET_NOTHING,
        "Gyroscope calibration callback called when internal target is not gyroscope. Internal target: %" PRIu8,
        handle->calibrationStage.target);
    SANITY_CHECK_C(handle->calibrationStage.state == BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_START,
        SANITY_CHECK_RET_NOTHING,
        "Gyroscope calibration callback called when internal state is not START. Internal state: %" PRIu8,
        handle->calibrationStage.state);

    if (code != BUSINESS_LAYER_CALIBRATION_GYROSCOPE_OK)
    {
        osal_logMsgE("Gyroscope calibration error in callback: %i", code);
    }

    handle->calibrationStage.state = (code == BUSINESS_LAYER_CALIBRATION_GYROSCOPE_OK)
                                         ? BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_FINISH
                                         : BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_ERROR;

    SANITY_CHECK_C(handle->userCallback != NULL, SANITY_CHECK_RET_NOTHING, "User callback is NULL");
    handle->userCallback(handle->userCallbackArg, handle->calibrationStage);

    handle->userCallback = NULL;
    handle->userCallbackArg = NULL;

    handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_FINISH;
}

/**
 * @brief Serve gyroscope calibration request
 *
 * @param[in,out] handle Calibration manager handle
 * @param[in] state Request calibration state
 *
 * @return BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_OK if successful, otherwise appropriate error code
 */
static businessLayer_calibrationManagerReturnCode_t serveGyroscopeCalibration(
    businessLayer_calibrationManager_t handle, businessLayer_calibrationManagerState_t state)
{
    SANITY_CHECK_C(handle != BUSINESS_LAYER_CALIBRATION_MANAGER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "Invalid calibration manager handle");

    SANITY_CHECK_ALWAYS_E(handle->calibrationStage.state == BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_FINISH,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_STATE,
        "Cant start gyroscope calibration. Expected internal state: %i, actual internal state: %i",
        BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_FINISH, handle->calibrationStage.state);
    SANITY_CHECK_ALWAYS_E(state == BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_START,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_STATE,
        "Cant start gyroscope calibration. Expected passed state: %i, actual passed state: %i",
        BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_START, state);

    const businessLayer_calibrationGyroscopeReturnCode_t code = businessLayer_calibrationGyroscopeRecalibrateAsync(
        handle->handles.gyroscope, gyroscopeCalibrationCallback, handle);
    SANITY_CHECK_ALWAYS_E(code == BUSINESS_LAYER_CALIBRATION_GYROSCOPE_OK,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INTERNAL_ERROR,
        "Failed to start gyroscope calibration, code: %i", code);

    handle->calibrationStage.state = state;

    return BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_OK;
}
static businessLayer_calibrationManagerReturnCode_t serveCalibration(
    businessLayer_calibrationManager_t handle, 
    const calibrationCommandPayload_t *request)
{
    SANITY_CHECK_C(handle != BUSINESS_LAYER_CALIBRATION_MANAGER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "Calibration manager handle is invalid");
    SANITY_CHECK_C(
        request != NULL, BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "Request pointer is NULL");

    switch (request->target)
    {
        case GENERAL_CALIBRATION_TARGET_MAGNETOMETER:
        {
            return serveMagnetometerCalibration(handle, request->state);
        }
        case GENERAL_CALIBRATION_TARGET_ACCELEROMETER:
        {
            return serveAccelerometerCalibration(handle, request->state);
        }
        case GENERAL_CALIBRATION_TARGET_GYROSCOPE:
        {
            return serveGyroscopeCalibration(handle, request->state);
        }
        default:
        {
            osal_logMsgE("Unknown request calibration target: %" PRIu8, request->target);
            return BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT;
        }
    }
}
businessLayer_calibrationManager_t businessLayer_calibrationManagerCreate(void)
{
    SANITY_CHECK_C(!internalHandle.isCreated, BUSINESS_LAYER_CALIBRATION_MANAGER_INVALID_HANDLE,
        "Calibration manager handle is already created");

    internalHandle.isCreated = true;

    return &internalHandle;
}
businessLayer_calibrationManagerReturnCode_t businessLayer_calibrationManagerDelete(
    businessLayer_calibrationManager_t handle)
{
    SANITY_CHECK_C(handle != BUSINESS_LAYER_CALIBRATION_MANAGER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "Invalid calibration manager handle");
    SANITY_CHECK_C(handle->isCreated, BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT,
        "Calibration manager handle not created");

    handle->isCreated = false;
    memset(handle, 0, sizeof(*handle));

    return BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_OK;
}
businessLayer_calibrationManagerReturnCode_t businessLayer_calibrationManagerOpen(
    businessLayer_calibrationManager_t handle, const businessLayer_calibrationManagerHandles_t *calibration_handles)
{
    SANITY_CHECK_C(handle != BUSINESS_LAYER_CALIBRATION_MANAGER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "Invalid calibration manager handle");
    SANITY_CHECK_C(calibration_handles != NULL, BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT,
        "Calibration handles pointer is NULL");
    SANITY_CHECK_C(calibration_handles->magnetometer != BUSINESS_LAYER_CALIBRATION_MAGNETOMETER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "Invalid magnetometer calibration handle");
    SANITY_CHECK_C(calibration_handles->accelerometer != BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "Invalid accelerometer calibration handle");
    SANITY_CHECK_C(calibration_handles->gyroscope != BUSINESS_LAYER_CALIBRATION_GYROSCOPE_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "Invalid gyroscope calibration handle");

    handle->handles = *calibration_handles;

    handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_FINISH;

    handle->pendingRequest.isUpdated = false;

    handle->wasCancelCallback = false;

    handle->userCallback = NULL;
    handle->userCallbackArg = NULL;

    PT_INIT(&handle->thread);

    return BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_OK;
}
businessLayer_calibrationManagerReturnCode_t businessLayer_calibrationManagerClose(
    businessLayer_calibrationManager_t handle)
{
    SANITY_CHECK_C(handle != BUSINESS_LAYER_CALIBRATION_MANAGER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "Invalid calibration manager handle");

    handle->handles.magnetometer = BUSINESS_LAYER_CALIBRATION_MAGNETOMETER_INVALID_HANDLE;
    handle->handles.accelerometer = BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_INVALID_HANDLE;
    handle->handles.gyroscope = BUSINESS_LAYER_CALIBRATION_GYROSCOPE_INVALID_HANDLE;

    handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_ERROR;

    handle->pendingRequest.isUpdated = false;

    handle->wasCancelCallback = false;

    handle->userCallback = NULL;
    handle->userCallbackArg = NULL;

    return BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_OK;
}

businessLayer_calibrationManagerReturnCode_t businessLayer_calibrationManagerServeRequestAsync(
    businessLayer_calibrationManager_t handle, const calibrationCommandPayload_t *request,
    businessLayer_calibrationManagerCallback_func_t callback, void *callback_arg)
{
    SANITY_CHECK_C(handle != BUSINESS_LAYER_CALIBRATION_MANAGER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "Invalid calibration manager handle");
    SANITY_CHECK_C(
        callback != NULL, BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_INVALID_ARGUMENT, "User callback is NULL");

    handle->pendingRequest.payload = *request;
    handle->pendingRequest.userCallback = callback;
    handle->pendingRequest.userCallbackArg = callback_arg;
    handle->pendingRequest.isUpdated = true;

    return BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_OK;
}

PT_THREAD(businessLayer_calibrationManagerThread(businessLayer_calibrationManager_t handle))
{
    SANITY_CHECK_C(handle != BUSINESS_LAYER_CALIBRATION_MANAGER_INVALID_HANDLE, PT_EXITED,
        "Calibration manager handle is invalid");

    struct pt *pt = &handle->thread;

    PT_BEGIN(pt);

    while (true)
    {
        PT_WAIT_UNTIL(pt, handle->pendingRequest.isUpdated);

        handle->pendingRequest.isUpdated = false;

        if (isAttemptToStartNewCalibrationWhileOtherIsInProgress(handle, &handle->pendingRequest.payload))
        {
            osal_logMsgW("Cancel %" PRIu8 " target calibration and start %" PRIu8 " target calibration",
                handle->calibrationStage.target, handle->pendingRequest.payload.target);

            handle->wasCancelCallback = false;
            const businessLayer_calibrationManagerReturnCode_t code = cancelCurrentCalibration(handle);
            CHECK_CANCEL_ERROR(handle, pt, code);

            PT_WAIT_UNTIL(pt, handle->wasCancelCallback);
            CHECK_CANCEL_ERROR(handle, pt, handle->cancelCode);

            handle->calibrationStage.state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_FINISH;
        }

        // Set user callbacks here, because of inconvenient accelerometer calibration API
        handle->userCallback = handle->pendingRequest.userCallback;
        handle->userCallbackArg = handle->pendingRequest.userCallbackArg;

        const businessLayer_calibrationManagerReturnCode_t code =
            serveCalibration(handle, &handle->pendingRequest.payload);
        if (code == BUSINESS_LAYER_CALIBRATION_MANAGER_RETURN_CODE_OK)
        {
            handle->calibrationStage.target = handle->pendingRequest.payload.target;
        }
        else
        {
            const calibrationCommandPayload_t error_response = {
                .target = handle->pendingRequest.payload.target,
                .state = BUSINESS_LAYER_CALIBRATION_MANAGER_STATE_ERROR,
            };
            handle->userCallback(handle->userCallback, error_response);

            handle->userCallback = NULL;
            handle->userCallbackArg = NULL;
        }
    }

    PT_END(pt);
}

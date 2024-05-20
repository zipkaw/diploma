
static inline 
PT_THREAD(
    calibrationAccelerometer_readParametersFromFlash(
        businessLayer_calibrationAccelerometer_t h))
{
    SANITY_CHECK_ALWAYS_E(h != BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_ERROR_INVALID_HANDLE, "Calibration handle is already NULL");

    businessLayer_calibrationAccelerometerInternal_t *calibHandle = h;

#ifdef ENABLE_FLASH
    businessLayer_flashReturnCode_t flashRetCode;
#endif

    struct pt *pt = &calibHandle->childPtThread;
    PT_BEGIN(pt);

#ifdef ENABLE_FLASH
    PT_WAIT_UNTIL(pt, businessLayer_flashReadyToOperate(calibHandle->flash));
    calibHandle->flashCb.flashIsReady = false;
    flashRetCode = businessLayer_flashReadAsync(calibHandle->flash, calibHandle->parametersAddress,
        &calibHandle->calibrationParams, sizeof(calibHandle->calibrationParams), calibrationAccelerometer_flashCb,
        calibHandle);

    if (flashRetCode != BUSINESS_LAYER_FLASH_RETURN_CODE_OK)
    {
        osal_logMsgE("Error while write into flash and equal %d", flashRetCode);
        calibHandle->requestToRead = CALIBRATION_ACCELEROMETER_PARAMETERS_WAS_READ_FAIL;
        PT_RESTART(pt);
    }
    // Wait until flash memory successfully read
    PT_WAIT_UNTIL(pt, calibHandle->flashCb.flashIsReady);

    if (calibHandle->flashCb.error != BUSINESS_LAYER_FLASH_RETURN_CODE_OK)
    {
        osal_logMsgE("Error from flash callback %d", calibHandle->flashCb.error);
        calibHandle->requestToRead = CALIBRATION_ACCELEROMETER_PARAMETERS_WAS_READ_FAIL;
        PT_RESTART(pt);
    }

    calibHandle->requestToRead = CALIBRATION_ACCELEROMETER_PARAMETERS_WAS_READ_SUCCESS;

    const uint32_t checkSum = calibHandle->calibrationParams.checkSum;
    const uint32_t outputCheckSum = businessLayer_crc32(BUSINESS_LAYER_CRC_INITIAL_VALUE,
        (uint8_t *)&calibHandle->calibrationParams, offsetof(calibrationAccelerometer_parameterWithCrc32_t, checkSum));

    if ((outputCheckSum != checkSum))
    {
        osal_logMsgW("Check sum 0x%" PRIx32 " does not equal with 0x%" PRIx32 ", default calibration matrix will set",
            outputCheckSum, checkSum);
        calibHandle->requestToRead = CALIBRATION_ACCELEROMETER_PARAMETERS_WAS_READ_FAIL;
        PT_EXIT(pt);
    }

    if (outputCheckSum == BUSINESS_LAYER_CRC_INVALID)
    {
        osal_logMsgW("Check sum is invalid and equal %" PRIx32 ", default calibration matrix will set", outputCheckSum);
        calibHandle->requestToRead = CALIBRATION_ACCELEROMETER_PARAMETERS_WAS_READ_FAIL;
        PT_EXIT(pt);
    }

#else
    calibHandle->requestToRead = CALIBRATION_ACCELEROMETER_PARAMETERS_WAS_READ_FAIL;
#endif

    PT_END(pt);
}
static inline businessLayer_flashReturnCode_t 
    calibrationAccelerometer_requestToWriteParametersIntoFlash(
    businessLayer_calibrationAccelerometer_t h)
{
    SANITY_CHECK_ALWAYS_E(h != BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_INVALID_HANDLE,
        BUSINESS_LAYER_FLASH_RETURN_CODE_INTERNAL_ERROR, "Calibration handle is already NULL");

#ifdef ENABLE_FLASH
    businessLayer_calibrationAccelerometerInternal_t *calibHandle = h;
    businessLayer_flashReturnCode_t flashRetCode;

    calibHandle->command = CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_APPLY_CALIBRATION;
    calibHandle->flashCb.flashIsReady = false;

    flashRetCode = businessLayer_flashWriteAsync(calibHandle->flash, calibHandle->parametersAddress,
        &(calibHandle->calibrationParams), sizeof(calibHandle->calibrationParams), calibrationAccelerometer_flashCb,
        calibHandle);
    return flashRetCode;
#else
    return BUSINESS_LAYER_FLASH_RETURN_CODE_INTERNAL_ERROR;
#endif
}
static PT_THREAD(
    calibrationAccelerometer_waitUntilMemsRead(struct pt *childPt, calibrationAccelerometer_memsCb_t *memsCb,
        businessLayer_calibrationAccelerometerWithDataCallback_func_t cbWithData, void *argWithData))
{
    PT_BEGIN(childPt);

    PT_WAIT_UNTIL(childPt, memsCb->memsWasRead);
    memsCb->memsWasRead = false;
    if (memsCb->error)
    {
        osal_logMsgE("Get error %d from mems", memsCb->error);
        cbWithData(argWithData, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_ERROR_MEMS, NULL);
        PT_RESTART(childPt);
    }
    PT_END(childPt);
}
static businessLayer_calibrationAccelerometerReturnCode_t 
    findMatrixInverse4x4(
    double *inputMatrix, double *outputMatrix)
{
    STATIC_ASSERT(CALIBRATION_ACCELEROMETER_MEASUREMENTS_ITS_TRANSPOSE_MAT_SIZE == 4 * 4, invalid_matrix_size);

    double inv[CALIBRATION_ACCELEROMETER_MEASUREMENTS_ITS_TRANSPOSE_MAT_SIZE];

    inv[0] = inputMatrix[5] * inputMatrix[10] * inputMatrix[15] - inputMatrix[5] * inputMatrix[11] * inputMatrix[14] -
             inputMatrix[9] * inputMatrix[6] * inputMatrix[15] + inputMatrix[9] * inputMatrix[7] * inputMatrix[14] +
             inputMatrix[13] * inputMatrix[6] * inputMatrix[11] - inputMatrix[13] * inputMatrix[7] * inputMatrix[10];

    inv[4] = -inputMatrix[4] * inputMatrix[10] * inputMatrix[15] + inputMatrix[4] * inputMatrix[11] * inputMatrix[14] +
             inputMatrix[8] * inputMatrix[6] * inputMatrix[15] - inputMatrix[8] * inputMatrix[7] * inputMatrix[14] -
             inputMatrix[12] * inputMatrix[6] * inputMatrix[11] + inputMatrix[12] * inputMatrix[7] * inputMatrix[10];

    inv[8] = inputMatrix[4] * inputMatrix[9] * inputMatrix[15] - inputMatrix[4] * inputMatrix[11] * inputMatrix[13] -
             inputMatrix[8] * inputMatrix[5] * inputMatrix[15] + inputMatrix[8] * inputMatrix[7] * inputMatrix[13] +
             inputMatrix[12] * inputMatrix[5] * inputMatrix[11] - inputMatrix[12] * inputMatrix[7] * inputMatrix[9];

    inv[12] = -inputMatrix[4] * inputMatrix[9] * inputMatrix[14] + inputMatrix[4] * inputMatrix[10] * inputMatrix[13] +
              inputMatrix[8] * inputMatrix[5] * inputMatrix[14] - inputMatrix[8] * inputMatrix[6] * inputMatrix[13] -
              inputMatrix[12] * inputMatrix[5] * inputMatrix[10] + inputMatrix[12] * inputMatrix[6] * inputMatrix[9];

    inv[1] = -inputMatrix[1] * inputMatrix[10] * inputMatrix[15] + inputMatrix[1] * inputMatrix[11] * inputMatrix[14] +
             inputMatrix[9] * inputMatrix[2] * inputMatrix[15] - inputMatrix[9] * inputMatrix[3] * inputMatrix[14] -
             inputMatrix[13] * inputMatrix[2] * inputMatrix[11] + inputMatrix[13] * inputMatrix[3] * inputMatrix[10];

    inv[5] = inputMatrix[0] * inputMatrix[10] * inputMatrix[15] - inputMatrix[0] * inputMatrix[11] * inputMatrix[14] -
             inputMatrix[8] * inputMatrix[2] * inputMatrix[15] + inputMatrix[8] * inputMatrix[3] * inputMatrix[14] +
             inputMatrix[12] * inputMatrix[2] * inputMatrix[11] - inputMatrix[12] * inputMatrix[3] * inputMatrix[10];

    inv[9] = -inputMatrix[0] * inputMatrix[9] * inputMatrix[15] + inputMatrix[0] * inputMatrix[11] * inputMatrix[13] +
             inputMatrix[8] * inputMatrix[1] * inputMatrix[15] - inputMatrix[8] * inputMatrix[3] * inputMatrix[13] -
             inputMatrix[12] * inputMatrix[1] * inputMatrix[11] + inputMatrix[12] * inputMatrix[3] * inputMatrix[9];

    inv[13] = inputMatrix[0] * inputMatrix[9] * inputMatrix[14] - inputMatrix[0] * inputMatrix[10] * inputMatrix[13] -
              inputMatrix[8] * inputMatrix[1] * inputMatrix[14] + inputMatrix[8] * inputMatrix[2] * inputMatrix[13] +
              inputMatrix[12] * inputMatrix[1] * inputMatrix[10] - inputMatrix[12] * inputMatrix[2] * inputMatrix[9];

    inv[2] = inputMatrix[1] * inputMatrix[6] * inputMatrix[15] - inputMatrix[1] * inputMatrix[7] * inputMatrix[14] -
             inputMatrix[5] * inputMatrix[2] * inputMatrix[15] + inputMatrix[5] * inputMatrix[3] * inputMatrix[14] +
             inputMatrix[13] * inputMatrix[2] * inputMatrix[7] - inputMatrix[13] * inputMatrix[3] * inputMatrix[6];

    inv[6] = -inputMatrix[0] * inputMatrix[6] * inputMatrix[15] + inputMatrix[0] * inputMatrix[7] * inputMatrix[14] +
             inputMatrix[4] * inputMatrix[2] * inputMatrix[15] - inputMatrix[4] * inputMatrix[3] * inputMatrix[14] -
             inputMatrix[12] * inputMatrix[2] * inputMatrix[7] + inputMatrix[12] * inputMatrix[3] * inputMatrix[6];

    inv[10] = inputMatrix[0] * inputMatrix[5] * inputMatrix[15] - inputMatrix[0] * inputMatrix[7] * inputMatrix[13] -
              inputMatrix[4] * inputMatrix[1] * inputMatrix[15] + inputMatrix[4] * inputMatrix[3] * inputMatrix[13] +
              inputMatrix[12] * inputMatrix[1] * inputMatrix[7] - inputMatrix[12] * inputMatrix[3] * inputMatrix[5];

    inv[14] = -inputMatrix[0] * inputMatrix[5] * inputMatrix[14] + inputMatrix[0] * inputMatrix[6] * inputMatrix[13] +
              inputMatrix[4] * inputMatrix[1] * inputMatrix[14] - inputMatrix[4] * inputMatrix[2] * inputMatrix[13] -
              inputMatrix[12] * inputMatrix[1] * inputMatrix[6] + inputMatrix[12] * inputMatrix[2] * inputMatrix[5];

    inv[3] = -inputMatrix[1] * inputMatrix[6] * inputMatrix[11] + inputMatrix[1] * inputMatrix[7] * inputMatrix[10] +
             inputMatrix[5] * inputMatrix[2] * inputMatrix[11] - inputMatrix[5] * inputMatrix[3] * inputMatrix[10] -
             inputMatrix[9] * inputMatrix[2] * inputMatrix[7] + inputMatrix[9] * inputMatrix[3] * inputMatrix[6];

    inv[7] = inputMatrix[0] * inputMatrix[6] * inputMatrix[11] - inputMatrix[0] * inputMatrix[7] * inputMatrix[10] -
             inputMatrix[4] * inputMatrix[2] * inputMatrix[11] + inputMatrix[4] * inputMatrix[3] * inputMatrix[10] +
             inputMatrix[8] * inputMatrix[2] * inputMatrix[7] - inputMatrix[8] * inputMatrix[3] * inputMatrix[6];

    inv[11] = -inputMatrix[0] * inputMatrix[5] * inputMatrix[11] + inputMatrix[0] * inputMatrix[7] * inputMatrix[9] +
              inputMatrix[4] * inputMatrix[1] * inputMatrix[11] - inputMatrix[4] * inputMatrix[3] * inputMatrix[9] -
              inputMatrix[8] * inputMatrix[1] * inputMatrix[7] + inputMatrix[8] * inputMatrix[3] * inputMatrix[5];

    inv[15] = inputMatrix[0] * inputMatrix[5] * inputMatrix[10] - inputMatrix[0] * inputMatrix[6] * inputMatrix[9] -
              inputMatrix[4] * inputMatrix[1] * inputMatrix[10] + inputMatrix[4] * inputMatrix[2] * inputMatrix[9] +
              inputMatrix[8] * inputMatrix[1] * inputMatrix[6] - inputMatrix[8] * inputMatrix[2] * inputMatrix[5];

    double det = inputMatrix[0] * inv[0] + inputMatrix[1] * inv[4] + inputMatrix[2] * inv[8] + inputMatrix[3] * inv[12];

    SANITY_CHECK_ALWAYS_E(fabs(det) > 1.0e-10, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_ERROR_MATH_OPERATION,
        "The determinant cannot be equal to 0");

    det = 1.0 / det;

    for (size_t i = 0; i < CALIBRATION_ACCELEROMETER_MEASUREMENTS_ITS_TRANSPOSE_MAT_SIZE; i++)
    {
        outputMatrix[i] = inv[i] * det;
    }

    return BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK;
}

static inline businessLayer_calibrationAccelerometerReturnCode_t 
    calibrationAccelerometer_constructExpectedValueMatrix(
    businessLayer_calibrationAccelerometer_t h)
{
    SANITY_CHECK_ALWAYS_E(h != BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_ERROR_INVALID_HANDLE, "Calibration handle is already NULL");

    businessLayer_calibrationAccelerometerInternal_t *calibHandle = h;
    calibrationAccelerometer_positionExpectedLinearAcceleration_t *expectedData =
        calibHandle->positionExpectedAcceleration;

    size_t sizeofExpectedMatrix = sizeof(expectedData[0]);
    memset(expectedData, 0, sizeofExpectedMatrix);

    size_t sizeOfOnePacket = sizeof(expectedData[0][0]) * CALIBRATION_ACCELEROMETER_NUM_OF_AXES;

    for (size_t i = 0; i < CALIBRATION_ACCELEROMETER_NUMBER_OF_POSITIONS; ++i)
    {
        for (size_t j = 0; j < CALIBRATION_ACCELEROMETER_NUM_OF_ONE_AMOUNT_PACKETS; ++j)
        {
            memcpy(&expectedData[i][j * CALIBRATION_ACCELEROMETER_NUM_OF_AXES],
                calibrationAccelerometer_expectedEachPosLinearAcceleration[i], sizeOfOnePacket);
        }
    }
    return BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK;
}
static businessLayer_calibrationAccelerometerReturnCode_t 
    calibrationAccelerometer_calculateScaleParameters(
    businessLayer_calibrationAccelerometer_t h)
{
    SANITY_CHECK_ALWAYS_E(h != BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_ERROR_INVALID_HANDLE, "Calibration handle is already NULL");
    businessLayer_calibrationAccelerometerInternal_t *calibHandle = h;
    calibrationAccelerometer_parameterWithCrc32_t *calibParams = &calibHandle->calibrationParams;
    double *raw_data = calibHandle->eachPositionData;

    calibrationAccelerometer_constructExpectedValueMatrix(h);
    double transposeXMatProduct[CALIBRATION_ACCELEROMETER_MEASUREMENTS_ITS_TRANSPOSE_MAT_SIZE];
    double inverseProduct[CALIBRATION_ACCELEROMETER_MEASUREMENTS_ITS_TRANSPOSE_MAT_SIZE];
    double transposeRawDataMat[CALIBRATION_ACCELEROMETER_NUM_OF_ALL_SAMPLES];

    Transpose_Matrix(transposeRawDataMat, raw_data, CALIBRATION_ACCELEROMETER_NUM_OF_ALL_PACKETS,
        CALIBRATION_ACCELEROMETER_INPUT_MEASUREMENTS_NUM);

    Multiply_Matrices(transposeXMatProduct, transposeRawDataMat, CALIBRATION_ACCELEROMETER_INPUT_MEASUREMENTS_NUM,
        CALIBRATION_ACCELEROMETER_NUM_OF_ALL_PACKETS, raw_data, CALIBRATION_ACCELEROMETER_INPUT_MEASUREMENTS_NUM);

    findMatrixInverse4x4(transposeXMatProduct, inverseProduct);

    double intermediateMultProduct[CALIBRATION_ACCELEROMETER_NUM_OF_ALL_SAMPLES];
    Multiply_Matrices(intermediateMultProduct, inverseProduct, CALIBRATION_ACCELEROMETER_INPUT_MEASUREMENTS_NUM,
        CALIBRATION_ACCELEROMETER_INPUT_MEASUREMENTS_NUM, transposeRawDataMat,
        CALIBRATION_ACCELEROMETER_NUM_OF_ALL_PACKETS);

    Multiply_Matrices(calibParams->scaleMatrix, intermediateMultProduct,
        CALIBRATION_ACCELEROMETER_INPUT_MEASUREMENTS_NUM, CALIBRATION_ACCELEROMETER_NUM_OF_ALL_PACKETS,
        (double *)calibHandle->positionExpectedAcceleration, CALIBRATION_ACCELEROMETER_NUM_OF_AXES);

    return BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK;
}

PT_THREAD(
    businessLayer_calibrationAccelerometerMainThread(
        businessLayer_calibrationAccelerometer_t h))
{
    SANITY_CHECK_ALWAYS_E(h != BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_INVALID_HANDLE,
        BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_ERROR_INVALID_HANDLE, "Calibration handle is already NULL");

    businessLayer_calibrationAccelerometerInternal_t *calibHandle = h;

    businessLayer_calibrationAccelerometerWithDataCallback_func_t cbWithData = calibHandle->cbParams.cbWithData;
    void *argWithData = calibHandle->cbParams.argWithData;
    businessLayer_calibrationAccelerometerWithoutDataCallback_func_t cbWithoutData =
        calibHandle->cbParams.cbWithoutData;
    void *argWithoutData = calibHandle->cbParams.argWithoutData;
    businessLayer_calibrationAccelerometerWithStateCallback_func_t cbWithState = calibHandle->cbParams.cbWithState;
    void *argWithState = calibHandle->cbParams.argWithState;

#ifdef ENABLE_FLASH
    businessLayer_flashReturnCode_t flashRetCode;
#endif

    struct pt *pt = &calibHandle->mainPtThread;

    double calibrateData[CALIBRATION_ACCELEROMETER_NUM_OF_AXES];
    double inputMeasurements[CALIBRATION_ACCELEROMETER_INPUT_MEASUREMENTS_NUM];

    PT_BEGIN(pt);
    while (true)
    {
        /** If income new command handle they */
        PT_SPAWN(pt, &calibHandle->childPtThread,
            calibrationAccelerometer_waitUntilMemsRead(
                &calibHandle->childPtThread, &calibHandle->memsCb, cbWithData, argWithData));
        calibrationAccelerometer_internalCommands_t command = calibHandle->command;

        /** If there is a request to read parameters, other commands
         * cannot be executed */
        if (calibHandle->requestToRead == CALIBRATION_ACCELEROMETER_REQUEST_FOR_PARAMETERS)
        {
            PT_SPAWN(pt, &calibHandle->childPtThread, calibrationAccelerometer_readParametersFromFlash(h));
            if (calibHandle->requestToRead == CALIBRATION_ACCELEROMETER_PARAMETERS_WAS_READ_FAIL)
            {
                osal_logMsgW("Error with flash or checksum after read. Default parameters matrix will set.");
                calibrationAccelerometer_setDefaultParams(h);
            }
        }
        else if (command == CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_APPLY_CALIBRATION)
        {
            calibrationAccelerometer_convertInputMeasurementsIntoDoubleArray(
                calibHandle->memsCb.data, inputMeasurements);
            // Using equation 3 from docs/accelerometer-calibration-stm.pdf
            Multiply_Matrices(calibrateData, inputMeasurements, CALIBRATION_ACCELEROMETER_INPUT_MEASUREMENTS_ROW_NUM,
                CALIBRATION_ACCELEROMETER_INPUT_MEASUREMENTS_NUM, calibHandle->calibrationParams.scaleMatrix,
                CALIBRATION_ACCELEROMETER_SCALE_MATRIX_COLS);
            calibrationAccelerometer_convertDoubleArrayIntoInputMeasurements(
                calibrateData, &(calibHandle->memsCb.data));

            cbWithData(argWithData, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK, &(calibHandle->memsCb.data));
        }
        else if (command == CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_RESET_TO_DEFAULTS)
        {
            /** Copy default matrix and write into flash memory */
            size_t scaleMatrixSize = sizeof(calibHandle->calibrationParams.scaleMatrix);
            memcpy(calibHandle->calibrationParams.scaleMatrix, calibrationAccelerometer_scaleDefaultMatrix,
                scaleMatrixSize);

            uint32_t checkSum =
                businessLayer_crc32(BUSINESS_LAYER_CRC_INITIAL_VALUE, (uint8_t *)&calibHandle->calibrationParams,
                    offsetof(calibrationAccelerometer_parameterWithCrc32_t, checkSum));
            if (checkSum == BUSINESS_LAYER_CRC_INVALID)
            {
                osal_logMsgE("Check sum is invalid and equal %" PRIx32, checkSum);
                cbWithoutData(argWithoutData, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_WARNING_INVALID_CHECKSUM);
            }
            calibHandle->calibrationParams.checkSum = checkSum;

#ifdef ENABLE_FLASH
            PT_WAIT_UNTIL(pt, businessLayer_flashReadyToOperate(calibHandle->flash));
            flashRetCode = calibrationAccelerometer_requestToWriteParametersIntoFlash(h);
            if (flashRetCode != BUSINESS_LAYER_FLASH_RETURN_CODE_OK)
            {
                osal_logMsgE("Error while write into flash and equal %d", flashRetCode);
                cbWithoutData(argWithoutData, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_ERROR_FLASH_OPERATION);
                calibHandle->command = CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_APPLY_CALIBRATION;
                PT_RESTART(pt);
            }
            // Wait until flash memory successfully read
            PT_WAIT_UNTIL(pt, calibHandle->flashCb.flashIsReady);
            if (calibHandle->flashCb.error != BUSINESS_LAYER_FLASH_RETURN_CODE_OK)
            {
                osal_logMsgE("Error from flash callback %d", calibHandle->flashCb.error);
                cbWithoutData(argWithoutData, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_ERROR_FLASH_OPERATION);
                calibHandle->command = CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_APPLY_CALIBRATION;
                PT_RESTART(pt);
            }
#else
            osal_logMsgE("Flash support was disabled on the firmware");
#endif

            cbWithoutData(argWithoutData, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK);
            calibHandle->command = CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_APPLY_CALIBRATION;
        }
        else if (command == CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_RECALIBRATE)
        {
            if (calibHandle->state == BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_POSITION_FINISH)
            {
                calibrationAccelerometer_calculateScaleParameters(h);
                calibHandle->command = CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_RECALIBRATE_CANCEL;
                uint32_t checkSum =
                    businessLayer_crc32(BUSINESS_LAYER_CRC_INITIAL_VALUE, (uint8_t *)&calibHandle->calibrationParams,
                        offsetof(calibrationAccelerometer_parameterWithCrc32_t, checkSum));
                if (checkSum == BUSINESS_LAYER_CRC_INVALID)
                {
                    osal_logMsgE("Check sum is invalid and equal %" PRIx32, checkSum);
                    cbWithState(argWithState, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_WARNING_INVALID_CHECKSUM,
                        calibHandle->state);
                }
                calibHandle->calibrationParams.checkSum = checkSum;

#ifdef ENABLE_FLASH
                PT_WAIT_UNTIL(pt, businessLayer_flashReadyToOperate(calibHandle->flash));
                flashRetCode = calibrationAccelerometer_requestToWriteParametersIntoFlash(h);
                if (flashRetCode != BUSINESS_LAYER_FLASH_RETURN_CODE_OK)
                {
                    osal_logMsgE("Error while write into flash and equal %d", flashRetCode);
                    calibHandle->state = BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_CALIBRATION_WAS_FINISHED;
                    cbWithState(argWithState, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_ERROR_FLASH_OPERATION,
                        calibHandle->state);
                    calibHandle->command = CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_APPLY_CALIBRATION;
                    calibHandle->cbParams.cbWithState = NULL;
                    PT_RESTART(pt);
                }
                // Wait until flash memory successfully read
                PT_WAIT_UNTIL(pt, calibHandle->flashCb.flashIsReady);
                if (calibHandle->flashCb.error != BUSINESS_LAYER_FLASH_RETURN_CODE_OK)
                {
                    osal_logMsgE("Error from flash callback %d", calibHandle->flashCb.error);
                    calibHandle->state = BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_CALIBRATION_WAS_FINISHED;
                    cbWithState(argWithState, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_ERROR_FLASH_OPERATION,
                        calibHandle->state);
                    calibHandle->command = CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_APPLY_CALIBRATION;
                    calibHandle->cbParams.cbWithState = NULL;
                    PT_RESTART(pt);
                }
#else
                osal_logMsgE("Flash support was disabled on the firmware");
#endif

                calibHandle->state = BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_CALIBRATION_WAS_FINISHED;
                cbWithState(argWithState, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK, calibHandle->state);

                calibHandle->cbParams.cbWithState = NULL;
            }
            else if (calibHandle->state == CALIBRATION_ACCELEROMETER_FIRST_POSITION)
            {
                PT_RESTART(pt);
            }
            else
            {
                osal_logMsgI("Recalibrate has not collect full data yet. Currently await collect data from %d position",
                    calibHandle->state);
                cbWithState(argWithState, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_ERROR_COMMAND, calibHandle->state);
                PT_RESTART(pt);
            }
        }
        else if (command == CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_SET_POS)
        {
            // check if collect all the data, if not wait until new position is set
            if (!(calibHandle->measurementCount < CALIBRATION_ACCELEROMETER_NUM_OF_ONE_AMOUNT_PACKETS) &&
                (calibHandle->state == CALIBRATION_ACCELEROMETER_LAST_POSITION))
            {
                calibHandle->command = CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_RECALIBRATE;
                calibHandle->state = BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_POSITION_FINISH;
                cbWithState(argWithState, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK, calibHandle->state);
                PT_RESTART(pt);
            }
            else if (calibHandle->measurementCount >= CALIBRATION_ACCELEROMETER_NUM_OF_ONE_AMOUNT_PACKETS)
            {
                businessLayer_calibrationAccelerometerStates_t nextPosition = calibHandle->state + 1;
                cbWithState(argWithState, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK, nextPosition);
                calibHandle->command = CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_WAIT_NEW_POS;
                PT_RESTART(pt);
            }
            size_t positionInStorage =
                (calibHandle->measurementCount * CALIBRATION_ACCELEROMETER_INPUT_MEASUREMENTS_NUM) +
                (CALIBRATION_ACCELEROMETER_NUM_OF_RAW_DATA * calibHandle->state);
            calibrationAccelerometer_convertInputMeasurementsIntoDoubleArray(
                calibHandle->memsCb.data, &calibHandle->eachPositionData[positionInStorage]);
            ++calibHandle->measurementCount;
        }
        else if (command == CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_RECALIBRATE_CANCEL)
        {
            calibHandle->measurementCount = 0;
            calibHandle->state = BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_CALIBRATION_WAS_FINISHED;
            calibHandle->command = CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_APPLY_CALIBRATION;
            cbWithoutData(argWithoutData, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_OK);
            cbWithState(argWithState, BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_CANCEL,
                BUSINESS_LAYER_CALIBRATION_ACCELEROMETER_CALIBRATION_WAS_FINISHED);
            calibHandle->cbParams.cbWithState = NULL;
        }
        else if (command == CALIBRATION_ACCELEROMETER_INTERNAL_COMMANDS_WAIT_NEW_POS)
        {
            PT_RESTART(pt);
        }
    }
    PT_END(pt);
}

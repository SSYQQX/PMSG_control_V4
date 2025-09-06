################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
BSP/EMIF/%.obj: ../BSP/EMIF/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/CCS12.3/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --fp_mode=relaxed --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/GPIO" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/I2C" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/eQEP" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/485_ModobusRTU" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/I2C" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/SPLL_FLL" --include_path="D:/controlSUITE/libs/app_libs/solar/v1.2/float/include" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/SPLL_SOGI" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/SPLL" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/RELAY" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/PID" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/driverlib" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/driverlib/inc" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/common/include" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/headers/include" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2" --include_path="D:/CCS12.3/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/EMIF" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/LED" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/EPWM" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/INT" --include_path="D:/CCS_WorkSpace/ThreePhaseRectifier_SpeedV4_2/BSP/ADC" --advice:performance=all --define=CPU1 -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="BSP/EMIF/$(basename $(<F)).d_raw" --obj_directory="BSP/EMIF" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '



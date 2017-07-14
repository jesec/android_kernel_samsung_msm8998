
dtb-$(CONFIG_SEC_DREAMQLTE_PROJECT) += msm8998-sec-dreamq-r00.dtb \
					msm8998-sec-dreamq-r01.dtb \
					msm8998-sec-dreamq-r02.dtb \
					msm8998-sec-dreamq-r03.dtb \
					msm8998-v2-sec-dreamq-r03.dtb \
					msm8998-sec-dreamq-r04.dtb \
					msm8998-sec-dreamq-r05.dtb \
					msm8998-sec-dreamq-r06.dtb \
					msm8998-sec-dreamq-r07.dtb \
					msm8998-sec-dreamq-r08.dtb \
					msm8998-sec-dreamq-r09.dtb \
					msm8998-sec-dreamq-r11.dtb \
					msm8998-sec-dreamq-r12.dtb

dtb-$(CONFIG_SEC_DREAM2QLTE_PROJECT) += msm8998-sec-dream2q-r00.dtb \
					msm8998-sec-dream2q-r01.dtb \
					msm8998-sec-dream2q-r02.dtb \
					msm8998-sec-dream2q-r03.dtb \
					msm8998-v2-sec-dream2q-r03.dtb \
					msm8998-sec-dream2q-r04.dtb \
					msm8998-sec-dream2q-r05.dtb \
					msm8998-sec-dream2q-r06.dtb \
					msm8998-sec-dream2q-r07.dtb \
					msm8998-sec-dream2q-r08.dtb \
					msm8998-sec-dream2q-r09.dtb \
					msm8998-sec-dream2q-r11.dtb \
					msm8998-sec-dream2q-r12.dtb

ifeq ($(filter y,$(CONFIG_MACH_DREAMQLTE_CHNZC) $(CONFIG_MACH_DREAMQLTE_CHNZM) $(CONFIG_MACH_DREAMQLTE_CHN_LDU)),y)
dtb-y = msm8998-sec-dreamq-chn-r03.dtb \
					msm8998-sec-dreamq-chn-r06.dtb \
					msm8998-sec-dreamq-chn-r07.dtb \
					msm8998-sec-dreamq-chn-r09.dtb \
					msm8998-sec-dreamq-chn-r12.dtb
endif

ifeq ($(filter y,$(CONFIG_MACH_DREAM2QLTE_CHNZC) $(CONFIG_MACH_DREAM2QLTE_CHN_LDU)),y)
dtb-y = msm8998-sec-dream2q-chn-r03.dtb \
					msm8998-sec-dream2q-chn-r06.dtb \
					msm8998-sec-dream2q-chn-r07.dtb \
					msm8998-sec-dream2q-chn-r09.dtb \
					msm8998-sec-dream2q-chn-r12.dtb
endif

ifeq ($(filter y,$(CONFIG_MACH_DREAMQLTE_CHNHK)),y)
dtb-y = msm8998-sec-dreamq-chnhk-r09.dtb \
					msm8998-sec-dreamq-chnhk-r12.dtb 
endif

ifeq ($(filter y,$(CONFIG_MACH_DREAM2QLTE_CHNHK)),y)
dtb-y = msm8998-sec-dream2q-chnhk-r09.dtb \
					msm8998-sec-dream2q-chnhk-r12.dtb 
endif

ifeq ($(filter y,$(CONFIG_MACH_DREAMQLTE_DCM) $(CONFIG_MACH_DREAMQLTE_KDI)),y)
	dtb-y = msm8998-sec-dreamq-jpn-r09.dtb \
					msm8998-sec-dreamq-jpn-r11.dtb \
					msm8998-sec-dreamq-jpn-r12.dtb	
endif

ifeq ($(filter y,$(CONFIG_MACH_DREAM2QLTE_DCM) $(CONFIG_MACH_DREAM2QLTE_KDI)),y)
dtb-y = msm8998-sec-dream2q-jpn-r07.dtb \
					msm8998-sec-dream2q-jpn-r09.dtb \
					msm8998-sec-dream2q-jpn-r11.dtb \
					msm8998-sec-dream2q-jpn-r12.dtb
endif


dtb-$(CONFIG_SEC_BAIKALQLTE_PROJECT) += msm8998-sec-baikalq-r00.dtb \
					 msm8998-sec-baikalq-r01.dtb

dtb-$(CONFIG_SEC_CRUISERLTE_PROJECT) += msm8998-sec-cruiser-r00.dtb 

ifeq ($(CONFIG_ARM64),y)
always          := $(dtb-y)
subdir-y        := $(dts-dirs)
else
targets += dtbs
targets += $(addprefix ../, $(dtb-y))

$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(dtb-y))
endif
clean-files := *.dtb *.reverse.dts


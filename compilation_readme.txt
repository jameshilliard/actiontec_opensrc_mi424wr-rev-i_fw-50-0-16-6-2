OpenRG 6.0.7.1.11 for Verizon Feroceon board Compilation
-------------------------------------------------------
To build the openrg.img (include TR143)
make config DIST=VERIZON_FEROCEON CONFIG_RG_PROD_IMG=y CONFIG_MV_WIFI_8864=m ACTION_TEC_VERIZON=y ACTION_TEC_TR143=y ACTION_TEC_TR143_VZ_PH2=y CONFIG_AEI_IPV6=y LIC=../jpkg_verizon_feroceon.lic && make

To build Jungo default GUI:
make config DIST=VERIZON_FEROCEON LIC=../jpkg_verizon_feroceon.lic && make

To build the openrg.img --Verizon GUI --HW-RevI --CONFIG_AEI_IPV6:
make config DIST=VERIZON_FEROCEON CONFIG_RG_PROD_IMG=y ACTION_TEC_VERIZON=y LIC=../jpkg_verizon_feroceon.lic CONFIG_AEI_IPV6=y && make

To build the openrg.img --Brighthouse GUI  --HW-RevI --CONFIG_AEI_IPV6:
make config DIST=VERIZON_FEROCEON CONFIG_RG_PROD_IMG=y ACTION_TEC_VERIZON=y ACTION_TEC_BRIGHTHOUSE=y LIC=../jpkg_verizon_feroceon.lic CONFIG_AEI_IPV6=y && make

/* build command change to the following start with release 50.0.15.3 */
To build the openrg.img --Brighthouse GUI  --HW-RevI --CONFIG_AEI_IPV6:
make config DIST=FEROCEON CONFIG_RG_PROD_IMG=y ACTION_TEC_BRIGHTHOUSE=y LIC=../jpkg_feroceon.lic CONFIG_AEI_IPV6=y && make

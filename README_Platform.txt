How to build Module for Platform
- It is only for modules are needed to using Android build system.
- Please check its own install information under its folder for other module.

[Step to build]
1. Get android open source.
    : version info - Android 10.0
    ( Download site : http://source.android.com )

2. Copy module that you want to build - to original android open source
   If same module exist in android open source, you should replace it. (no overwrite)
   
  # It is possible to build all modules at once.
  
3. You should add module name to 'PRODUCT_PACKAGES' in 'build\target\product\core.mk' as following case.
	case 1) e2fsprog : should add 'e2fsck','resize2fs' to PRODUCT_PACKAGES
	case 2) libexifa : should add 'libexifa.camera.samsung' to PRODUCT_PACKAGES
	case 3) libjpega : should add 'libjpega.camera.samsung' to PRODUCT_PACKAGES
	case 4) keyutils : should add 'libknox_keyutils' to PRODUCT_PACKAGES
	

ex.) [build\target\product\core.mk] - add all module name for case 1 ~ 5 at once
    
# e2fsprog
PRODUCT_PACKAGES += \
    e2fsck \
    libext2fs \
    libext2_blkid \
    ibext2_e2p \
    resize2fs
    
# libexifa
PRODUCT_PACKAGES += \
    libexifa.camera.samsung
    
# libjpega
PRODUCT_PACKAGES += \
    libjpega.camera.samsung
    
# KeyUtils
PRODUCT_PACKAGES += \
    libknox_keyutils
   
4. excute build command
   ./build_64bit.sh

5. Note : 
   To download the source code of S/W listed below, please visit http://opensource.samsung.com and find "Mobile -> Mobile Application" menu, 
   and then, you will be able to download what you want. 
   You might save time in finding the right one by making use of the search keyword below. 
	- ShareLive.apk : "ShareLive"
	- MdecService.apk : "MdecService"
	- BixbyWakeup.apk : "BixbyWakeup"
	- AREmoji.apk : "AREmoji"
	- VoiceNote_5.0.apk : "Voice Recorder"
	- SecEmail_P.apk : "Email"
	- HoneyBoard.apk : "HoneyBoard"
	- DeXonPC.apk : "DeXonPC"
	- SmartSwitch.apk : "Smart Switch"
	- SamsungCamera.apk : "Camera"
	- Fmm.apk : "FMM"
	- KidsHome.apk : "KidsHome"
	- Notes33.apk : "Samsung Notes"
	- LiveStickers.apk : "LiveStickers"
	- SamsungMessages_11.apk : "Messaging"
	- SBrowser_11.0_Removable.apk : "SBrowser"
	- SamsungConnect.apk : "SamsungConnect"

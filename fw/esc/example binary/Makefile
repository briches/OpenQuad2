
.PHONY: flash

flash:
	jlink -SelectEmuBySN 801020691 -device STM32F031C6 -if SWD -speed 8000 -autoconnect 1 -ExitOnError -CommanderScript ./flash.jlink


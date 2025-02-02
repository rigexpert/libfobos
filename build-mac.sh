mkdir -p build/mac

g++ -w -fpermissive -Wno-permissive \
    -Ifobos/ \
    -L/opt/homebrew/lib -lusb-1.0 \
    -o build/mac/fobos_devinfo eval/fobos_devinfo_main.c fobos/fobos.c

g++ -w -fpermissive -Wno-permissive \
    -Ifobos/ \
    -L/opt/homebrew/lib -lusb-1.0 \
    -o build/mac/fobos_fwloader eval/fobos_fwloader_main.c fobos/fobos.c

g++ -w -fpermissive -Wno-permissive \
    -Ifobos/ -I./ \
    -L/opt/homebrew/lib -lusb-1.0 \
    -o build/mac/fobos_recorder eval/fobos_recorder_main.c wav/wav_file.c fobos/fobos.c

g++ -w -fpermissive -Wno-permissive -shared -fPIC \
    -L/opt/homebrew/lib -lusb-1.0 \
    -o build/mac/fobos.dylib fobos/fobos.c


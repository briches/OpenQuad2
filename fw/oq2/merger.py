main_app = esc_app = zero_pad = "" 

flash_start = 0x08000000
storage_address = 0x080C0000
storage_offset = storage_address - flash_start

with open("_build/oq2.bin", "rb") as fp: 
    main_app = fp.read() 
    print(type(main_app))

main_app_len = len(main_app)
print("Main app is ", main_app_len, " bytes")
  
# Reading data from file2 
with open("../esc/_build/blinky.bin", "rb") as fp: 
    esc_app = fp.read() 

esc_app_len = len(esc_app)
print("ESC app is ", esc_app_len, " bytes") 

padding_len = storage_offset - main_app_len
print("Padding is ", padding_len, " bytes") 

if padding_len < 0:
    print("Main app overflows into storage by ", -padding_len, " bytes")
    exit(-1)

padding = bytes(padding_len)

# Merging 2 files 
# To add the data of file2 
# from next line CALL
output = main_app + padding + esc_app
  
with open ("_build/output.bin", "wb") as fp: 
    fp.write(output) 
private static final char[] HEX_ARRAY = "0123456789ABCDEF".toCharArray();


public static String bytesToHexString(byte[] bytes, int numbytes) 
{
    char[] hexChars = new char[numbytes * 3];

    for (int j = 0; j < numbytes; j++) 
    {
        int v = bytes[j] & 0xFF;
        hexChars[j * 3] = HEX_ARRAY[v >>> 4];
        hexChars[j * 3 + 1] = HEX_ARRAY[v & 0x0F];
        hexChars[j * 3 + 2] = ' ';
    }
    return new String(hexChars);
}
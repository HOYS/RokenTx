# c16_to_i8_shift.py
import numpy as np, sys
fin = sys.argv[1] if len(sys.argv)>1 else "BBD_0001.C16"
fout = sys.argv[2] if len(sys.argv)>2 else "BBD_0001_i8_shift.raw"

arr = np.fromfile(fin, dtype=np.int16, count=-1)
I16 = arr[0::2]; Q16 = arr[1::2]
# Convert to int8 by arithmetic right shift (>>8), matching typical “truncate” behavior
I8  = np.clip((I16 >> 8), -128, 127).astype(np.int8)
Q8  = np.clip((Q16 >> 8), -128, 127).astype(np.int8)
out = np.empty(I8.size*2, np.int8); out[0::2]=I8; out[1::2]=Q8
out.tofile(fout)
print("Wrote", fout, "samples:", I8.size)

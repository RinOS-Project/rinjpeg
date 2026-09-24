# RinJPEG

RinJPEG is a small, header-only C decoder used by RinImage for a bounded JPEG
profile. It is not a general-purpose or fully JPEG-compatible replacement for
libjpeg.

## Supported API

Include `rinjpeg.h`. The `static inline` API provides `rjpeg_get_info` and
`rjpeg_decode`; `_with_scratch` variants accept caller-owned decoder state.
Decode writes numeric `0xAARRGGBB` words into caller-owned storage.

## Supported and unsupported profiles

The decoder accepts 8-bit baseline sequential DCT (SOF0) images with one
grayscale component or three color components. Three-component images support
4:4:4, 4:2:2, and 4:2:0 sampling. Restart intervals, quantization/Huffman
tables, and ordinary APP/COM marker segments are parsed.

Progressive SOF2, arithmetic or other JPEG processes, non-8-bit precision,
other component counts, and other sampling layouts return an unsupported or
data error. The decoder does not expose EXIF, ICC, XMP, orientation, or a
complete color-management contract. The presence of JPEG markers in the header
does not imply that those profiles are decoded.

## Ownership, limits, and errors

Encoded input, pixel output, and optional decoder scratch are borrowed for the
call. `rjpeg_decode` checks the supplied pixel capacity and maximum dimensions;
the caller must still allocate the complete output buffer. Independent calls
with independent buffers are reentrant. The convenience API holds decoder
tables in a local `RJpegDecoder`; users with constrained stacks should use the
caller-scratch entry point.

The decoder caps encoded input at 64 MiB, either dimension at 8192, and total
pixels at 16,777,216. `RJPEG_OK`, `RJPEG_UNSUPPORTED`, `RJPEG_DATA_ERROR`, and
`RJPEG_ERROR` distinguish success, unsupported profile, malformed data, and
invalid/capacity failure. The direct API is allocation-free but does not
provide a CPU deadline or cancellation callback.

## Security, ABI, build, and tests

Validate untrusted input with the decoder and keep the dimension/pixel limits
appropriate for the caller. The decoder retains no input or output pointers
after return. This is a header-defined source API with no separately versioned
binary ABI guarantee. RinOS integrates it through the parent RinImage build;
this repository has no standalone build or test target.

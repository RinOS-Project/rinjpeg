# RinJPEG

RinJPEG is a small, header-only C decoder used by RinImage for a bounded JPEG
profile. It is not a general-purpose or fully JPEG-compatible replacement for
libjpeg. The public header is C11-compatible as well as usable from C++ callers.

## Supported API

Include `rinjpeg.h`. The `static inline` API provides `rjpeg_get_info` and
`rjpeg_decode`; `_with_scratch` variants accept caller-owned decoder state.
`rjpeg_get_info_cancellable` and `rjpeg_decode_cancellable` accept an optional
caller-owned cancellation predicate. Header parsing polls while walking marker
fill bytes, entropy decoding polls at MCU-row boundaries, and output conversion
polls per pixel row. Cancellation returns `RJPEG_CANCELLED`. Decode writes
numeric `0xAARRGGBB` words into caller-owned storage.

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
pixels at 16,777,216. `RJPEG_OK`, `RJPEG_UNSUPPORTED`, `RJPEG_DATA_ERROR`,
`RJPEG_ERROR`, and `RJPEG_CANCELLED` distinguish success, unsupported profile,
malformed data, invalid/capacity failure, and caller cancellation. The direct
API is allocation-free. Cancellation callbacks run synchronously on the
calling thread and should be bounded and side-effect safe.

## Security, ABI, build, and tests

Validate untrusted input with the decoder and keep the dimension/pixel limits
appropriate for the caller. The decoder retains no input or output pointers
after return. This is a header-defined source API with no separately versioned
binary ABI guarantee. RinOS integrates it through the parent RinImage build;
this repository has no standalone build or test target.

## Public API contract

| Requirement | Contract |
| --- | --- |
| Purpose | RinJPEG is RinOS's bounded decoder for a documented baseline JPEG subset, intended for use through RinImage or directly through its C interface. |
| Supported API | The public C interface is `rinjpeg.h`. It decodes 8-bit baseline SOF0 grayscale or three-component JPEG with supported 4:4:4, 4:2:2, and 4:2:0 sampling; cancellable entry points return `RJPEG_CANCELLED` when their caller predicate requests cancellation. |
| Unsupported API | Progressive JPEG, other frame types, unsupported component layouts, and formats other than JPEG are rejected. |
| ownership | The caller owns the input bytes and destination buffer and keeps them valid for the call. The decoder does not retain either buffer. |
| thread-safety | Independent calls with separate input and output buffers may run concurrently. Do not share writable output buffers between calls. |
| limits | Input is limited to 64 MiB; width and height to 8192 each; decoded pixels to 16,777,216. Exceeding a limit is rejected. |
| errors | Invalid, truncated, unsupported, over-limit, or cancelled input returns a failure status; callers must not use output unless the call succeeds. |
| ABI stability | The C declarations in `rinjpeg.h` are the public ABI. No ABI stability guarantee is currently published; consumers should rebuild against the version they use. |
| security | Treat JPEG bytes as untrusted. The decoder applies size and dimension limits and cancellable callers can bound marker, MCU-row, and output-row work; callers remain responsible for checking status and surrounding work. |
| build | No standalone build entry point is provided. RinImage is the supported integration point in the RinOS build. |
| test | No standalone test command is provided by this repository. RinImage integration tests, when present in the consuming tree, are the relevant validation. |

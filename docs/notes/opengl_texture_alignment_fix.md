# OpenGL Texture Alignment Issue and Fix

## Overview

This document explains a critical issue that was discovered in the QuickViz Canvas implementation where certain image dimensions would cause application crashes during texture upload to OpenGL, and describes the implemented fix.

## Problem Description

### Symptoms
- Application crashes with segmentation fault during `glTexImage2D` calls
- Crashes occurred specifically with images having dimensions 1203×809 pixels
- Other dimensions like 1756×1152 or 800×538 worked perfectly fine
- The issue was consistent across different image formats (PNG) and content

### Initial Investigation
The crash occurred in `Canvas::SetupBackgroundImage()` specifically during the OpenGL texture upload phase:

```cpp
glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, 
             0, format, GL_UNSIGNED_BYTE, data);
```

The STB image loading was successful - the crash happened during GPU texture creation.

## Root Cause Analysis

### The Real Issue: OpenGL Row Alignment

The root cause was **OpenGL row alignment requirements**. OpenGL expects texture data to be aligned to specific byte boundaries, controlled by the `GL_UNPACK_ALIGNMENT` parameter (default: 4 bytes).

### Mathematical Analysis

For RGB images (3 bytes per pixel):
- **1203 × 3 = 3609 bytes per row**
- **3609 % 4 = 1** (not aligned to 4-byte boundary) ❌

Working images:
- **1756 × 3 = 5268 bytes per row**  
- **5268 % 4 = 0** (aligned) ✅
- **800 × 3 = 2400 bytes per row**
- **2400 % 4 = 0** (aligned) ✅

### Why This Causes Crashes

When `GL_UNPACK_ALIGNMENT` is set to 4 (default), OpenGL expects each row of texture data to start at a 4-byte aligned memory address. For unaligned data:

1. OpenGL may read beyond the allocated memory boundary
2. This can cause segmentation faults or memory corruption
3. The behavior is undefined and driver-dependent

## Solution Implementation

### The Fix

A generic solution was implemented that automatically detects and handles alignment issues:

```cpp
// Check for OpenGL row alignment issues
// OpenGL expects texture data to be aligned to 4-byte boundaries by default
int bytes_per_pixel = (format == GL_RGBA) ? 4 : (format == GL_RGB) ? 3 : 1;
int bytes_per_row = width * bytes_per_pixel;
bool alignment_issue = (bytes_per_row % 4) != 0;

if (alignment_issue) {
  std::cout << "Canvas: Adjusting alignment for " << width << "x" << height 
            << " texture (row size: " << bytes_per_row << " bytes)" << std::endl;
  // Set pixel alignment to 1 byte to handle unaligned row data
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
}

glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height,
             0, format, GL_UNSIGNED_BYTE, data);

// Restore default alignment if we changed it
if (alignment_issue) {
  glPixelStorei(GL_UNPACK_ALIGNMENT, 4);  // Restore default
}
```

### How the Fix Works

1. **Detection**: Calculate bytes per row and check if divisible by 4
2. **Adjustment**: Set `GL_UNPACK_ALIGNMENT` to 1 for problematic textures
3. **Upload**: Perform normal texture upload with adjusted alignment
4. **Restoration**: Restore default alignment setting

### Why GL_UNPACK_ALIGNMENT = 1 Works

Setting `GL_UNPACK_ALIGNMENT` to 1 tells OpenGL:
- "Don't assume any row alignment"
- "Read pixel data byte-by-byte without alignment requirements"
- This works for any row size but may have minor performance implications

## Technical Details

### OpenGL Alignment Parameters

- `GL_UNPACK_ALIGNMENT`: Controls alignment for data read from client memory (default: 4)
- `GL_PACK_ALIGNMENT`: Controls alignment for data written to client memory (default: 4)
- Valid values: 1, 2, 4, 8 bytes

### Performance Considerations

- Aligned textures (4-byte rows) have optimal performance
- Unaligned textures with `GL_UNPACK_ALIGNMENT=1` work correctly but may be slightly slower
- The performance difference is negligible for most applications

### Affected Image Formats

This issue affects images where `width × channels % 4 ≠ 0`:

| Format | Problematic Width Examples |
|--------|---------------------------|
| RGB (3 channels) | 1203, 1205, 1207, 1209, ... (any width where width × 3 % 4 ≠ 0) |
| Grayscale (1 channel) | Any non-multiple of 4 |
| RGBA (4 channels) | Never problematic (4 × anything is divisible by 4) |

## Alternative Solutions Considered

### 1. Image Resizing (Workaround)
- Resize problematic images to aligned dimensions
- **Pros**: Guaranteed compatibility
- **Cons**: Requires manual intervention, data loss

### 2. Row Padding (Memory Solution)
- Add padding bytes to make rows 4-byte aligned
- **Pros**: Maintains optimal performance
- **Cons**: Complex implementation, memory overhead

### 3. Generic Alignment Fix (Chosen Solution)
- Automatically adjust `GL_UNPACK_ALIGNMENT` when needed
- **Pros**: Automatic, generic, no data loss
- **Cons**: Minor performance impact for unaligned textures

## Testing and Validation

### Test Cases
1. **Original problematic images**: Now load successfully
2. **Working images**: Continue to work without alignment adjustment
3. **Various dimensions**: Tested multiple alignment scenarios

### Verification
The fix was verified with:
- Original 1203×809 images (both PNG variants)
- Working 1756×1152 and 800×538 images
- Edge cases with different channel counts

## Best Practices

### For Image Assets
1. **Prefer aligned dimensions** when possible for optimal performance
2. **RGB images**: Use widths that make `width × 3` divisible by 4
3. **RGBA images**: Any width works (always aligned)

### For Developers
1. **Don't hard-code dimension checks** - use generic alignment detection
2. **Always restore OpenGL state** after temporary changes
3. **Log alignment adjustments** for debugging purposes

## Code Location

The fix is implemented in:
- **File**: `src/renderer/src/renderable/canvas.cpp`
- **Function**: `Canvas::SetupBackgroundImage()`
- **Lines**: Texture upload section (~415-440)

## Future Enhancements

Potential improvements:
1. **Global alignment policy**: Allow configuration of default alignment behavior
2. **Performance metrics**: Measure alignment impact in real applications
3. **Memory optimization**: Implement row padding for frequently used textures

## References

- [OpenGL specification on pixel storage](https://www.khronos.org/opengl/wiki/Pixel_Transfer#Pixel_layout)
- [GL_UNPACK_ALIGNMENT documentation](https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glPixelStore.xhtml)
- [STB Image library](https://github.com/nothings/stb)

---

**Author**: Claude Code Assistant  
**Date**: July 2025  
**Version**: 1.0
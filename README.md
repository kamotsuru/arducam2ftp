# ArduCAM to FTP
## 0. Motivation
I just wanted to copy an image taken by my ArduCAM-Mini-2MP with ESP32 to my server whenever two successive images significantly differs from each other, but most examples only show how to store the images to an SD card...

## 1. Take an ArduCAM image as jpg format
The following example includes taking a jpg image by ArduCAM-Mini-2MP and storing it to a buffer.

https://github.com/Dasch0/esp32-arducam-edge-impulse/blob/main/src/main.cpp


## 2. Decode a jpg image and calculate the difference between two images
Tiny JPEG Decompressor can decode a jpg image with a small footprint. Then, I decided to use this for each pixel in the decoded jpg image being compared between two successive images.

http://elm-chan.org/fsw/tjpgd/en/appnote.html

The struct *IODEV* in Example above is modified so as to pass a char type list of jpg image taken by ArduCAM instead of FILE type pointer as an input stream *\*jbuf* and the size as a member *lbyte*. Additionally, a member *diff* to keep the sum of differences between same positional pixels of two successive images.
```
// TJpgDec Session identifier for input/output functions (name, members and usage are as user defined)
typedef struct {
    char *jbuf;             /* Input stream */
    uint8_t *fbuf;          /* Output frame buffer */
    unsigned int wfbuf;     /* Width of the frame buffer [pix] */
    uint32_t lbyte;         /* Number bytes left in the input stream */
    int diff;               /* difference between current and previous frame buffers */
} IODEV;
```

Similarly, the input function above is modified so as to copy the above *\*jbuf* to the read buffer *buff* every *nbyte* by memcpy.
```
// TJpgDec User defined input function
size_t in_func (    /* Returns number of bytes read (zero on error) */
    JDEC* jd,       /* Decompression object */
    uint8_t* buff,  /* Pointer to the read buffer (null to remove data) */
    size_t nbyte    /* Number of bytes to read/remove */
)
{
    IODEV *dev = (IODEV*)jd->device;   /* Session identifier (5th argument of jd_prepare function) */

    /* Copy input stream, dev->jbuf, to read buffer, buff */
    if (nbyte > dev->lbyte)
      nbyte = dev->lbyte;
    if (buff) /* Read data from imput stream */
      memcpy(buff, dev->jbuf, nbyte);
    dev->lbyte -= nbyte;
    dev->jbuf += nbyte;

    return nbyte;
}
```

The output function above is also modified so as to keep the sum of square root of the sum of the squares of differences of the same positional pixels between two successive images, i.e. the decoded bitmap and the existing output frame buffer, into the above *diff* at the same time of copying the decoded bitmap *bitmap* to the output frame buffer *wfbuf*.
```
// TJpgDec User defined output function
unsigned int out_func (      /* Returns 1 to continue, 0 to abort */
    JDEC* jd,       /* Decompression object */
    void* bitmap,   /* Bitmap data to be output */
    JRECT* rect     /* Rectangular region of output image */
)
{
    IODEV *dev = (IODEV*)jd->device;   /* Session identifier (5th argument of jd_prepare function) */
    uint8_t *src, *dst;
    uint16_t y, bws, i;
    unsigned int bwd;

    /* Progress indicator */
    if (rect->left == 0) {
        if(Serial)
          Serial.printf("\r%lu%%. ", (rect->top << jd->scale) * 100UL / jd->height);
    }
    /* Copy the output image rectangle to the frame buffer */
    src = (uint8_t*)bitmap;                           /* Output bitmap */
    dst = dev->fbuf + N_BPP * (rect->top * dev->wfbuf + rect->left);  /* Left-top of rectangle in the frame buffer */
    bws = N_BPP * (rect->right - rect->left + 1);     /* Width of the rectangle [byte] */
    bwd = N_BPP * dev->wfbuf;                         /* Width of the frame buffer [byte] */
    for (y = rect->top; y <= rect->bottom; y++) {
      for (i = 0; i < bws/N_BPP; i++){  /* Calculate the difference between output bitmap and previous frame buffer before copy */
        dev->diff += sq((int)dst[i*3] - (int)src[i*3]);
        dev->diff += sq((int)dst[i*3 + 1] - (int)src[i*3 + 1]);
        dev->diff += sq((int)dst[i*3 + 2] - (int)src[i*3 + 2]);
      }
      memcpy(dst, src, bws);   /* Copy a line */
      src += bws; dst += bwd;  /* Next line */
    }

    return 1;    /* Continue to decompress */
}
```

## 3. Put a jpg image to an FTP server
The following is to copy a particular object to an FTP server.

https://playground.arduino.cc/Code/FTP/

Whenever the *diff* by the above calculation exceeds a certain threshold, 20000000 here, the newer image is stored into a ftp server.
```
  if (devid.diff > 20000000) { // ftp the current jpg image if the difference between current and previous and images is more than this value
    doFTP(jpeg_buffer, jpeg_size);
    ...
  }
```

## 4. Put a expected jpg image to an FTP server
I combined these three examples into one and can send expected jpg images, i.e. with significant difference between two successive images, to an FTP server as follows.

https://github.com/kamotsuru/arducam2ftp/blob/main/arducam2ftp.ino
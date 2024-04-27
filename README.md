sum of differences between same positional pixels of two successive images.
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
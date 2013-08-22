;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; CUB2TDU,INP [,/NOBYTSCL] [,TDU=TDU] [,MAXWIDTH=MAXWIDTH]
;
; 2D data (inpdata) to TDU
; INP:  represents input 2D data
;       - STRING:  open file "inp" as CUB (readisis.pro)
;       - OTHER NUMERIC (byte, integer, long, float, double):
;         - bytscl(inp>0)    - if /NOBYTSCL keyword is not specified
;         - byte(inp>0)    - if /NOBYTSCL keyword is specified
;
; /NOBYTSCL:  see INP above
;
; TDU=TDU:  device e.g. /dev/tdu1
;           - default = /dev/tdu0
;
; MAXWIDTH=MAXWIdTH:  maximum output width of device
;                     - default = 1728 (e.g. TDU-850)
;                     - data will be shrunk to this width if it is wider
;                     - data will be stretched to this width if it is less 
;                       than 2/3 as wide
;                       
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
pro cub2tdu,inp,nobytscl=nobytscl,tdu=tdu,maxwidth=maxwidth

; default TDU configuration parameters

tdudev = '/dev/tdu0'	; device name
tduwid = 1728L		; maximum width

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; test if user specified TDU device name

sz = size(tdu)				; user specifies TDU device name
if sz(sz(0)+1) eq 7 then begin
  tdudev = strtrim(tdu(0))
endif else begin
  if sz(sz(0)+1) ne 0 then begin
    message,'TDU device name must be a string'
  endif
endelse

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; test if user specified TDU device max width

sz = size(maxwidth)
if sz(sz(0)+1) ne 0 then begin
  tduwid = long(maxwidth(0))
endif

;;;;;;;;;;;;
; input data

sz=size(inp)
if sz(sz(0)+1) eq 7 then begin		; check for string => CUB filename
  lcldata = 0L
  readisis,inp,lcldata
endif else begin			; not string, assume numeric data
  lcldata = inp
endelse

szl = size(lcldata)			; test for < 2D
if szl(0) lt 2 then begin
  help,input
  help,lcldata
  message, 'data is not 2D or greater'
endif

if szl(0) gt 2 then begin		; test for > 2D
  message,'using first plane only',/continue
  lcldata = lcldata(*,*)
endif

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; geometric stretch or shrink as necessary

szl = size(lcldata)

w = long(szl(1))
if w gt tduwid OR w lt ((tduwid*2)/3) then neww = tduwid else neww = w

if w ne neww then begin
  newr = (long(szl(2)) * neww) / w
  message,'resizing data to '+strtrim(string(neww),2)+' columns',/continue
  lcldata = congrid( lcldata, neww, newr, /interp)
endif

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; radiometric stretch if requested

if keyword_set(nobytscl) then begin
  szl = size(lcldata)
  if szl(sz(0)+1) ne 1 then lcldata = byte(lcldata>0)
endif else begin
  lcldata = bytscl(lcldata>0)
endelse

;;;;;;;;;;;;;;;;;;;
; send bytes to TDU

szl = size(lcldata)

cmd = 'tunetdu ' + tdudev + ' -r ' + strtrim(string(szl(1))) + ' > /dev/null'

spawn,cmd			; set width

lun=-1L
openw,lun,tdudev,/get_lun	; open device
writeu,lun,lcldata		; write bytes
free_lun,lun			; close device

end

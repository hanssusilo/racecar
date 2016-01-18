; Auto-generated. Do not edit!


(cl:in-package racecar-msg)


;//! \htmlinclude wd.msg.html

(cl:defclass <wd> (roslisp-msg-protocol:ros-message)
  ((a_r
    :reader a_r
    :initarg :a_r
    :type cl:float
    :initform 0.0)
   (b_r
    :reader b_r
    :initarg :b_r
    :type cl:float
    :initform 0.0)
   (a_l
    :reader a_l
    :initarg :a_l
    :type cl:float
    :initform 0.0)
   (b_l
    :reader b_l
    :initarg :b_l
    :type cl:float
    :initform 0.0))
)

(cl:defclass wd (<wd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name racecar-msg:<wd> is deprecated: use racecar-msg:wd instead.")))

(cl:ensure-generic-function 'a_r-val :lambda-list '(m))
(cl:defmethod a_r-val ((m <wd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar-msg:a_r-val is deprecated.  Use racecar-msg:a_r instead.")
  (a_r m))

(cl:ensure-generic-function 'b_r-val :lambda-list '(m))
(cl:defmethod b_r-val ((m <wd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar-msg:b_r-val is deprecated.  Use racecar-msg:b_r instead.")
  (b_r m))

(cl:ensure-generic-function 'a_l-val :lambda-list '(m))
(cl:defmethod a_l-val ((m <wd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar-msg:a_l-val is deprecated.  Use racecar-msg:a_l instead.")
  (a_l m))

(cl:ensure-generic-function 'b_l-val :lambda-list '(m))
(cl:defmethod b_l-val ((m <wd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar-msg:b_l-val is deprecated.  Use racecar-msg:b_l instead.")
  (b_l m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wd>) ostream)
  "Serializes a message object of type '<wd>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_l))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_l))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wd>) istream)
  "Deserializes a message object of type '<wd>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_r) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_r) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_l) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_l) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wd>)))
  "Returns string type for a message object of type '<wd>"
  "racecar/wd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wd)))
  "Returns string type for a message object of type 'wd"
  "racecar/wd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wd>)))
  "Returns md5sum for a message object of type '<wd>"
  "0eceb51e340e9bf10c7c57a66c32a9c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wd)))
  "Returns md5sum for a message object of type 'wd"
  "0eceb51e340e9bf10c7c57a66c32a9c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wd>)))
  "Returns full string definition for message of type '<wd>"
  (cl:format cl:nil "# Lines describing the walls~%# Right wall: a_r*x + b_r~%# Left wall: a_l*x + b_l~%~%float32 a_r~%float32 b_r~%float32 a_l~%float32 b_l~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wd)))
  "Returns full string definition for message of type 'wd"
  (cl:format cl:nil "# Lines describing the walls~%# Right wall: a_r*x + b_r~%# Left wall: a_l*x + b_l~%~%float32 a_r~%float32 b_r~%float32 a_l~%float32 b_l~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wd>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wd>))
  "Converts a ROS message object to a list"
  (cl:list 'wd
    (cl:cons ':a_r (a_r msg))
    (cl:cons ':b_r (b_r msg))
    (cl:cons ':a_l (a_l msg))
    (cl:cons ':b_l (b_l msg))
))

;; Auto-generated. Do not edit!


(when (boundp 'mymsg::neighborpos)
  (if (not (find-package "MYMSG"))
    (make-package "MYMSG"))
  (shadow 'neighborpos (find-package "MYMSG")))
(unless (find-package "MYMSG::NEIGHBORPOS")
  (make-package "MYMSG::NEIGHBORPOS"))

(in-package "ROS")
;;//! \htmlinclude neighborpos.msg.html


(defclass mymsg::neighborpos
  :super ros::object
  :slots (_xpos _ypos _time_stamp _id ))

(defmethod mymsg::neighborpos
  (:init
   (&key
    ((:xpos __xpos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:ypos __ypos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:time_stamp __time_stamp) 0.0)
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _xpos __xpos)
   (setq _ypos __ypos)
   (setq _time_stamp (float __time_stamp))
   (setq _id (round __id))
   self)
  (:xpos
   (&optional __xpos)
   (if __xpos (setq _xpos __xpos)) _xpos)
  (:ypos
   (&optional __ypos)
   (if __ypos (setq _ypos __ypos)) _ypos)
  (:time_stamp
   (&optional __time_stamp)
   (if __time_stamp (setq _time_stamp __time_stamp)) _time_stamp)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; float64[] _xpos
    (* 8    (length _xpos)) 4
    ;; float64[] _ypos
    (* 8    (length _ypos)) 4
    ;; float64 _time_stamp
    8
    ;; int32 _id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _xpos
     (write-long (length _xpos) s)
     (dotimes (i (length _xpos))
       (sys::poke (elt _xpos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _ypos
     (write-long (length _ypos) s)
     (dotimes (i (length _ypos))
       (sys::poke (elt _ypos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _time_stamp
       (sys::poke _time_stamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _id
       (write-long _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _xpos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _xpos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _xpos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _ypos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ypos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _ypos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _time_stamp
     (setq _time_stamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mymsg::neighborpos :md5sum-) "2b4b6c25254ec4f08814442766cebb75")
(setf (get mymsg::neighborpos :datatype-) "mymsg/neighborpos")
(setf (get mymsg::neighborpos :definition-)
      "float64[] xpos
float64[] ypos
float64 time_stamp
int32 id

")



(provide :mymsg/neighborpos "2b4b6c25254ec4f08814442766cebb75")



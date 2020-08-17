;; Auto-generated. Do not edit!


(when (boundp 'open_quadruped::JointAngles)
  (if (not (find-package "OPEN_QUADRUPED"))
    (make-package "OPEN_QUADRUPED"))
  (shadow 'JointAngles (find-package "OPEN_QUADRUPED")))
(unless (find-package "OPEN_QUADRUPED::JOINTANGLES")
  (make-package "OPEN_QUADRUPED::JOINTANGLES"))

(in-package "ROS")
;;//! \htmlinclude JointAngles.msg.html


(defclass open_quadruped::JointAngles
  :super ros::object
  :slots (_fl _fr _bl _br ))

(defmethod open_quadruped::JointAngles
  (:init
   (&key
    ((:fl __fl) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:fr __fr) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:bl __bl) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:br __br) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _fl __fl)
   (setq _fr __fr)
   (setq _bl __bl)
   (setq _br __br)
   self)
  (:fl
   (&optional __fl)
   (if __fl (setq _fl __fl)) _fl)
  (:fr
   (&optional __fr)
   (if __fr (setq _fr __fr)) _fr)
  (:bl
   (&optional __bl)
   (if __bl (setq _bl __bl)) _bl)
  (:br
   (&optional __br)
   (if __br (setq _br __br)) _br)
  (:serialization-length
   ()
   (+
    ;; float32[] _fl
    (* 4    (length _fl)) 4
    ;; float32[] _fr
    (* 4    (length _fr)) 4
    ;; float32[] _bl
    (* 4    (length _bl)) 4
    ;; float32[] _br
    (* 4    (length _br)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _fl
     (write-long (length _fl) s)
     (dotimes (i (length _fl))
       (sys::poke (elt _fl i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _fr
     (write-long (length _fr) s)
     (dotimes (i (length _fr))
       (sys::poke (elt _fr i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _bl
     (write-long (length _bl) s)
     (dotimes (i (length _bl))
       (sys::poke (elt _bl i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _br
     (write-long (length _br) s)
     (dotimes (i (length _br))
       (sys::poke (elt _br i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _fl
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _fl (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _fl i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _fr
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _fr (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _fr i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _bl
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bl (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _bl i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _br
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _br (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _br i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get open_quadruped::JointAngles :md5sum-) "cc955e0566b06523084e350c65b2944e")
(setf (get open_quadruped::JointAngles :datatype-) "open_quadruped/JointAngles")
(setf (get open_quadruped::JointAngles :definition-)
      "float32[] fl
float32[] fr
float32[] bl
float32[] br

")



(provide :open_quadruped/JointAngles "cc955e0566b06523084e350c65b2944e")



(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-wrapper.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(require "models/arrow-object.l")


(defun setup-robot-for-hrp2jsknts-reach-button
    ()
  (setq *robot* (instance hrp2jsknts-robot :init))
  ;; assoc finger-tip-coords
  (unless (and (send (send *robot* :hand :rarm) :get :finger-tip-coords)
               (send (send *robot* :hand :larm) :get :finger-tip-coords))
    (send (send *robot* :hand :rarm)
          :put :finger-tip-coords
          (make-cascoords
           :parent (send (send *robot* :rarm :end-coords) :parent)
           :coords
           (send (send (car (last (send (send *robot* :hand :rarm) :links))) :copy-worldcoords)
                 :transform (make-coords :pos #f(-20 40 -40) :rpy (float-vector 0 0 (/ -pi 8))))
           ))
    (send (send *robot* :hand :larm)
          :put :finger-tip-coords
          (make-cascoords
           :parent (send (send *robot* :larm :end-coords) :parent)
           :coords
           (send (send (car (last (send (send *robot* :hand :larm) :links))) :copy-worldcoords)
                 :transform (make-coords :pos #f(-20 -40 -40) :rpy (float-vector pi 0 (/ -pi 8))))
           ))
    )
  ;; disable min-max table
  (dolist (jnt (send *robot* :joint-list))
    (send jnt :set-val 'joint-min-max-table nil)
    (send jnt :set-val 'joint-min-max-target nil)
    )
  ;; set narrow limit of min/max angle instead of min-max table
  (send (send *robot* :joint "RLEG_JOINT0") :min-angle -35)
  (send (send *robot* :joint "LLEG_JOINT0") :max-angle 35)
  (send (send *robot* :joint "RLEG_JOINT2") :min-angle -70)
  (send (send *robot* :joint "LLEG_JOINT2") :min-angle -70)
  (send (send *robot* :joint "CHEST_JOINT1") :min-angle -1)
  (send (send *robot* :joint "CHEST_JOINT1") :max-angle 42)
  (send (send *robot* :joint "RARM_JOINT5") :min-angle -50)
  (send (send *robot* :joint "RARM_JOINT5") :max-angle 50)
  (send (send *robot* :joint "RARM_JOINT6") :min-angle -50)
  (send (send *robot* :joint "RARM_JOINT6") :max-angle 50)
  (send (send *robot* :joint "LARM_JOINT5") :min-angle -50)
  (send (send *robot* :joint "LARM_JOINT5") :max-angle 50)
  (send (send *robot* :joint "LARM_JOINT6") :min-angle -50)
  (send (send *robot* :joint "LARM_JOINT6") :max-angle 50)
  )

(defun setup-environment-for-hrp2jsknts-reach-button
    ()
  (require :sample "./demo-hrp2jsknts-reach-button-setting.l")
  (unless (boundp '*pc-convex-hull*)
    (let* ((ground-height-tol 200)
           )
      (setq *pc-convex-hull*
            (convex-hull-3d
             (remove-if
              #'(lambda (p) (< (elt p 2) ground-height-tol))
              (send *pc* :point-list))))
      ))
  )

(defun hrp2jsknts-reach-button
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     &allow-other-keys
     )
  (setup-robot-for-hrp2jsknts-reach-button)
  (setup-environment-for-hrp2jsknts-reach-button)
  (objects (list *robot* *pc*))

  ;; (send *robot* :reset-pose)
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  (setq *target-coords-list*
        (remove
         nil
         (mapcar
          #'(lambda (i x) (if (= (mod i 20) 0) x))
          (range (length *co2*))
          *co2*
          )))
  (setq
   *ret*
   (send *robot*
         :inverse-kinematics-trajectory-optmotiongen
         *target-coords-list*
         :move-target-list (list (send (send *robot* :hand :rarm) :get :finger-tip-coords))
         :debug-view (list :additional-draw-objects (list *pc*))
         :rotation-axis-list (make-list (length *target-coords-list*) :initial-element (list t))
         :thre-list (make-list (length *target-coords-list*) :initial-element (list 10))
         :rthre-list (make-list (length *target-coords-list*) :initial-element (list (deg2rad 5)))
         :root-virtual-mode :planar
         :root-virtual-joint-invariant? t
         :pre-process-func pre-process-func
         :post-process-func post-process-func
         :adjacent-regular-scale 1e-4
         :collision-avoidance-link-pair
         (mapcar #'(lambda (l) (list l *pc-convex-hull*)) (flatten (send *robot* :arms)))
         :collision-distance-limit 100 ;; [mm]
         :revert-if-fail nil
         ))
  (objects (append (list *robot* *pc*) *target-coords-list*))
  (visualize-trajectory-result *ret* :visualize-callback-func visualize-callback-func)
  *ret*
  )
(warn "(hrp2jsknts-reach-button)~%")

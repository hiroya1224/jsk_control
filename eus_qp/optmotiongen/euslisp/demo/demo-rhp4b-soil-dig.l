(load "package://eus_qp/optmotiongen/euslisp/trajectory-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(require :rhp4b "package://khi_ros_bridge_tutorials/euslisp/rhp4b-interface.l")
(require "models/arrow-object.l")


(defun make-shovel
    (&key (shaft-length 1010.0) (shaft-radius 18.5)
          (handle-width 126.0) (handle-radius 17.5) (handle-radius-2 7.0)
          (handle-length 155.0)
          (blade-width 250.0) (blade-width-2 230.0) (blade-length 300.0)
          (blade-angle 0.500)
          )
  (let (blade
        shaft
        handle
        shovel
        )
    (setq blade (make-prism
                 (list
                  (float-vector (/ blade-length 2.0) (/ blade-width 2.0) (- 3.0))
                  (float-vector (- (/ blade-length 2.0)) (/ blade-width-2 2.0) (- 3.0))
                  (float-vector (- (/ blade-length 2.0)) (- (/ blade-width-2 2.0)) (- 3.0))
                  (float-vector (/ blade-length 2.0) (- (/ blade-width 2.0)) (- 3.0))
                  )
                 (float-vector 0 0 6.0)))
    (setq shaft (make-cylinder shaft-radius shaft-length :segments 18))
    (let ((handle-base
           (make-prism
            (list
             (float-vector 0 (- shaft-radius) (- handle-radius))
             (float-vector (- handle-length) (- (/ handle-width 2.0)) (- handle-radius))
             (float-vector (- handle-length)    (/ handle-width 2.0)  (- handle-radius))
             (float-vector 0 shaft-radius (- handle-radius))
             )
            (float-vector 0 0 (* 2 handle-radius)))
           )
          (handle-hole
           (make-prism
            (list
             (float-vector (- (- (* 2 handle-radius)) 0) (- shaft-radius) (- (* 2 handle-radius)))
             (float-vector (+ (- handle-length)
                              (* 2 handle-radius))
                           (-
                            (/ (* (- (/ handle-width 2) (* 2 handle-radius-2))
                                  (- handle-length (* 4 handle-radius-2)))
                               handle-length))
                           (- (* 2 handle-radius)))
             (float-vector (+ (- handle-length)
                              (* 2 handle-radius))
                           (/ (* (- (/ handle-width 2) (* 2 handle-radius-2))
                                 (- handle-length (* 4 handle-radius-2)))
                              handle-length)
                           (- (* 2 handle-radius)))
             (float-vector (- (* 2 handle-radius)) shaft-radius (- (* 2 handle-radius)))
             )
            (float-vector 0 0 (* 4 handle-radius)))
           )
          )
      (setq handle (body- handle-base handle-hole))
      )
    (send shaft :rotate pi/2 :y)
    (send blade :rotate blade-angle :y)

    ;; Colorize
    (send handle :set-color #f(0.65 0.26 0.16))
    (send shaft  :set-color (scale (/ 1 255.0) #f(222 184 135)))
    (send blade  :set-color (scale (/ 1 255.0) #f(112 128 144)))

    (let ((gap (/ (- blade-length (* blade-length (cos blade-angle))) 2.0)))
      (send blade :translate (float-vector (- (+ shaft-length (/ blade-length 2.0)) gap) 0 (- (* (/ blade-length 2.0) (sin blade-angle)))) :world))
    (send handle :assoc shaft)
    (send handle :assoc blade)
    (send handle :rotate pi/2 :x)
    (send handle :translate (float-vector (- handle-length handle-radius) 0 0))
    (setq shovel (instance bodyset-link :init (make-cascoords)
                           :bodies (list handle shaft blade) :name :shovel))
    (let ((rcoords (make-cascoords :angle -pi/2 :axis :x))
          (lcoords (make-cascoords :angle -pi/2 :axis :y))
          (grasp-near (make-cascoords :angle -pi/2 :axis :x))
          (grasp-far (make-cascoords :angle -pi/2 :axis :x))
          (target  (make-cascoords :angle -pi/2 :axis :x))
          (edge    (make-cascoords :angle -pi/2 :axis :x))
          (lshoulder (make-cascoords :angle -pi/2 :axis :x))
          (rshoulder (make-cascoords :angle -pi/2 :axis :x))
          (compromise-point (make-cascoords :angle -pi/2 :axis :x))) ;levelするためのworking-point
      (send shovel :put :rarm rcoords)
      (send shovel :put :larm lcoords)
      (send shovel :put :graspnear grasp-near)
      (send shovel :put :graspfar grasp-far)
      (send shovel :put :target target)
      (send shovel :put :edge edge)
      (send shovel :put :lshoulder lshoulder)
      (send shovel :put :rshoulder rshoulder)
      (send shovel :put :compromise-point compromise-point)

      (send target :translate (float-vector (+ handle-length shaft-length (- handle-radius) (* (/ blade-length 2.0) (cos blade-angle))) (* (/ blade-length 2.0) (sin blade-angle)) 0) :world)
      (send target :rotate (- blade-angle) :y)

      (send lcoords :translate (float-vector 0 0 (- (/ (+ handle-length shaft-length) 2.0))))
      (send grasp-near :translate (float-vector handle-length 0 0) :local)
      (send grasp-far :translate (float-vector (+ shaft-length handle-length -50) 0 0) :local)

      (send edge :translate (float-vector (+ handle-length shaft-length (- handle-radius) (* blade-length (cos blade-angle))) (* blade-length (sin blade-angle)) 0) :world)
      (send edge :rotate (- blade-angle) :y)

      (send lshoulder :translate (float-vector (+ handle-length shaft-length (- handle-radius)) 0 (- (* blade-width 0.35))))
      (send lshoulder :rotate (- blade-angle) :y)

      (send rshoulder :translate (float-vector (+ handle-length shaft-length (- handle-radius)) 0 (* blade-width 0.35)))
      (send rshoulder :rotate (- blade-angle) :y)

      (send compromise-point :translate (float-vector (+ handle-length shaft-length (- handle-radius) (* 0.75 blade-length (cos blade-angle))) (* 0.2 blade-length (sin blade-angle)) 0) :world)

      (send shovel :assoc rcoords)
      (send shovel :assoc lcoords)
      (send shovel :assoc grasp-near)
      (send shovel :assoc grasp-far)
      (send shovel :assoc target)
      (send shovel :assoc edge)
      (send shovel :assoc lshoulder)
      (send shovel :assoc rshoulder)
      (send shovel :assoc compromise-point)
      )
    shovel
    ))

(defun setup-rhp4b-for-demo-rhp4b-soil-dig
    ()
  (setq *robot* (instance rhp4b-robot :init))
  ;; assoc shovel
  ;; you can get
  ;; shovel link by (send *robot* :link :shovel)
  ;; shovel virtual joint by (send *robot* :joint :shovel-virtual-joint)
  (let* ((shovel)
         (shovel-virtual-joint)
         )
    (setq shovel (make-shovel))
    (send shovel :move-coords (send *robot* :rarm-end-coords) (send shovel :get :rarm))
    (send shovel :rotate pi/2 :x)
    (send (send (send *robot* :rarm-end-coords) :parent) :assoc shovel)
    (setq shovel-virtual-joint
          (instance rotational-joint :init
                    :name :shovel-virtual-joint
                    :parent-link (send (send *robot* :rarm-end-coords) :parent)
                    :child-link shovel :axis :z :min -20 :max 20))
    (send shovel :add-joint shovel-virtual-joint)
    (send shovel :add-parent-link (send (send *robot* :rarm-end-coords) :parent))
    (send (send (send *robot* :rarm-end-coords) :parent) :add-child-links shovel)
    (send *robot* :set-val 'links
          (append (send *robot* :get-val 'links) (list shovel)))
    (send *robot* :set-val 'joint-list
          (append (send *robot* :get-val 'joint-list) (list shovel-virtual-joint)))
    (send *robot* :set-val 'bodies
          (append (send *robot* :get-val 'bodies) (send shovel :bodies)))
    )
  ;; assoc contact coords
  (send *robot* :put :rleg-contact-coords
        (make-cascoords :name :rleg-contact-coords
                        :parent (send *robot* :link "R_ANKLE_P_LINK")
                        :coords (send (send (send *robot* :rleg-end-coords) :copy-worldcoords)
                                      :translate (float-vector 0 0 30))
                        ))
  (send *robot* :put :lleg-contact-coords
        (make-cascoords :name :lleg-contact-coords
                        :parent (send *robot* :link "L_ANKLE_P_LINK")
                        :coords (send (send (send *robot* :lleg-end-coords) :copy-worldcoords)
                                      :translate (float-vector 0 0 30))
                        ))
  ;; create support polygon
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send *robot* :link "L_WRIST_Y_LINK") :thre 10.0)
  (create-support-polygon *robot* (send *robot* :link "R_WRIST_Y_LINK") :thre 10.0)
  (create-support-polygon *robot* (send *robot* :link "L_ANKLE_P_LINK") :thre 10.0)
  (create-support-polygon *robot* (send *robot* :link "R_ANKLE_P_LINK") :thre 10.0)
  ;; disable min-max table
  (dolist (jnt (send *robot* :joint-list))
    (send jnt :set-val 'joint-min-max-table nil)
    (send jnt :set-val 'joint-min-max-target nil)
    )
  ;; overwrite max/min-angle
  (send (send *robot* :joint "L_CROTCH_Y") :max-angle 60)
  (send (send *robot* :joint "L_CROTCH_Y") :min-angle -60)
  (send (send *robot* :joint "R_CROTCH_Y") :max-angle 60)
  (send (send *robot* :joint "R_CROTCH_Y") :min-angle -60)
  (send (send *robot* :joint "L_SHOULDER_P") :max-angle 60)
  (send (send *robot* :joint "L_SHOULDER_P") :min-angle -120)
  (send (send *robot* :joint "R_SHOULDER_P") :max-angle 60)
  (send (send *robot* :joint "R_SHOULDER_P") :min-angle -120)
  )

(defun setup-contact-constraint-for-demo-rhp4b-soil-dig
    (&key
     (mu-trans 0.3)
     (mu-rot 0.01)
     (max-fz 500)
     (support-polygon-margin 20)
     )
  (send (send *robot-env* :robot :get :rleg-contact-coords)
        :put :contact-constraint
        (instance* default-contact-constraint
                   :init
                   :mu-trans mu-trans
                   :mu-rot mu-rot
                   :max-fz max-fz
                   (let* ((poly
                           (send *robot-env* :robot :get :r_ankle_p_link-support-polygon))
                          (vs
                           (mapcar #'(lambda (v)
                                       (send (send (send *robot-env* :robot :get :rleg-contact-coords) :worldcoords)
                                             :inverse-transform-vector v))
                                   (send poly :vertices)
                                   ))
                          )
                     (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) support-polygon-margin)
                           :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) support-polygon-margin)
                           :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) support-polygon-margin)
                           :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) support-polygon-margin)
                           ))
                   ))
  (send (send *robot-env* :robot :get :lleg-contact-coords)
        :put :contact-constraint
        (instance* default-contact-constraint
                   :init
                   :mu-trans mu-trans
                   :mu-rot mu-rot
                   :max-fz max-fz
                   (let* ((poly
                           (send *robot-env* :robot :get :l_ankle_p_link-support-polygon))
                          (vs
                           (mapcar #'(lambda (v)
                                       (send (send (send *robot-env* :robot :get :lleg-contact-coords) :worldcoords)
                                             :inverse-transform-vector v))
                                   (send poly :vertices)
                                   ))
                          )
                     (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) support-polygon-margin)
                           :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) support-polygon-margin)
                           :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) support-polygon-margin)
                           :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) support-polygon-margin)
                           ))
                   ))
  )

(defun demo-rhp4b-soil-dig-trajectory
    (&key
     (optimize-torque? nil)
     (optimize-start-end-torque? nil)
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? t)
     &allow-other-keys
     )
  ;; setup target coords and target arrows
  (setq *rleg-coords* (make-coords :pos (float-vector 0 -105 0)))
  (setq *lleg-coords* (make-coords :pos (float-vector 0 105 0)))
  (setq *arrow-list* (mapcar #'(lambda (x) (arrow)) (make-list 2)))
  (send (elt *arrow-list* 0) :newcoords (make-coords :pos (float-vector 1000 0 0)))
  (send (elt *arrow-list* 1) :newcoords (make-coords :pos (float-vector 1000 0 200)))
  ;; setup robot
  (setup-rhp4b-for-demo-rhp4b-soil-dig)
  (setq *contact-list*
        (list
         (instance contact-ik-arg :init
                   :name :target0
                   :target-coords (elt *arrow-list* 0)
                   :translation-axis t
                   :rotation-axis (list :z :y)
                   :transform-coords (elt *arrow-list* 0)
                   )
         (instance contact-ik-arg :init
                   :name :target1
                   :target-coords (elt *arrow-list* 1)
                   :translation-axis t
                   :rotation-axis (list :z :y)
                   :transform-coords (elt *arrow-list* 1)
                   )
         ))
  (setq *robot-env* (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
  (send *robot-env* :invariant-joint-list
        (list (send *robot-env* :joint :shovel-virtual-joint)))
  (send *robot-env* :variant-joint-list
        (set-difference (send *robot-env* :joint-list) (send *robot-env* :invariant-joint-list)))
  (setup-contact-constraint-for-demo-rhp4b-soil-dig)
  (send *robot-env* :robot :reset-pose)
  (setq *robot-env-list* nil)
  (setq *instant-config-task-list* nil)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :first
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if (or optimize-torque? optimize-start-end-torque?) (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list (send *rleg-coords* :copy-worldcoords)
                   (send *lleg-coords* :copy-worldcoords)
                   (send (send (car *robot-env-list*) :contact :target0) :contact-coords)
                   (send (send (car *robot-env-list*) :robot :link :shovel) :get :larm)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (send (car *robot-env-list*) :robot :link :shovel) :get :edge)
                   (send (car *robot-env-list*) :robot :larm-end-coords)
                   )
             :contact-target-coords-list
             (list (send *rleg-coords* :copy-worldcoords)
                   (send *lleg-coords* :copy-worldcoords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   )
             ;; :posture-joint-list
             ;; (send (car *robot-env-list*) :robot :joint-list)
             ;; :posture-joint-angle-list
             ;; (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :first
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list (send *rleg-coords* :copy-worldcoords)
                   (send *lleg-coords* :copy-worldcoords)
                   (send (send (car *robot-env-list*) :contact :target1) :contact-coords)
                   (send (send (car *robot-env-list*) :robot :link :shovel) :get :larm)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (send (car *robot-env-list*) :robot :link :shovel) :get :edge)
                   (send (car *robot-env-list*) :robot :larm-end-coords)
                   )
             :contact-target-coords-list
             (list (send *rleg-coords* :copy-worldcoords)
                   (send *lleg-coords* :copy-worldcoords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   )
             ;; :posture-joint-list
             ;; (send (car *robot-env-list*) :robot :joint-list)
             ;; :posture-joint-angle-list
             ;; (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             )
   *instant-config-task-list*
   )

  (objects (append *robot-env-list* *arrow-list*))

  (setq *trajectory-config-task*
        (instance* trajectory-configuration-task :init
                   :instant-config-task-list (reverse *instant-config-task-list*)
                   :norm-regular-scale-max 1e-1
                   :adjacent-regular-scale 1e-2
                   (if optimize-torque?
                       (list
                        :norm-regular-scale-max 1e-4
                        :norm-regular-scale-offset 1e-7
                        :adjacent-regular-scale 2e-3
                        :torque-regular-scale 1e-4
                        ))
                   ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *trajectory-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (mapcar
                        ;;  #'(lambda (config-task) (validate-element-jacobian :config-task config-task))
                        ;;  ;; #'(lambda (config-task) (validate-jacobian :config-task config-task))
                        ;;  (send (send sqp :config-task) :instant-config-task-list))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num 50)

  (objects (append (list *robot-env*) *arrow-list*))
  (send *trajectory-config-task* :play-animation
        :robot-env *robot-env*
        :loop? visualize-loop?
        :visualize-callback-func visualize-callback-func
        )
  )
(warn "(demo-rhp4b-soil-dig-trajectory)~%")

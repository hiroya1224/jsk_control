;; -*- mode: EusLisp; coding: utf-8-unix; -*-

(require :arrow-object "models/arrow-object.l")
(require :ik-statics-optmotiongen "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(require :ik-optmotiongen "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-wrapper.l")

(defvar *robot*)
(defvar *shovel*)
(defvar *shovel-virtual-joint*)
(defvar *sandbox*)

(defun make-sandbox (&key (width 1500) (depth 2000) (height 300) (thick 70)
                          (sand-height 200) (color #f(0 0 1)))
  (let ((front-side (make-cube thick width height :pos (float-vector 0 0 (/ height 2))))
        (back-side  (make-cube thick width height :pos (float-vector depth 0 (/ height 2))))
        (right-side (make-cube depth thick height :pos (float-vector (/ depth 2) (/ width -2) (/ height 2))))
        (left-side  (make-cube depth thick height :pos (float-vector (/ depth 2) (/ width 2) (/ height 2))))
        (sand       (make-cube depth width sand-height :pos (float-vector (/ depth 2) 0 (/ sand-height 2)))))

    (send-all (list front-side back-side right-side left-side) :set-color color)
    (send sand :set-color #f(0.45 0.26 0.16))

    (dolist (side (list front-side back-side right-side left-side))
      (send sand :assoc side))

    (setq sandbox (instance bodyset :init (make-cascoords)
                            :bodies (list sand front-side back-side right-side left-side)))
    (send sandbox :put :front-side front-side)
    (send sandbox :put :front-face (send front-side :face 5))
    (send sandbox :put :sand-face (send sand :face 1))
    sandbox
    ))

(defun make-shovel (&key (shaft-length 1010.0) (shaft-radius 18.5)
                         (handle-width 126.0) (handle-radius 17.5) (handle-radius-2 7.0)
                         (handle-length 155.0)
                         (blade-width 250.0) (blade-width-2 230.0) (blade-length 300.0)
                         (blade-angle 0.500)
                         )
  (let (
        blade
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
    (send shaft  :set-color (normalize-decimal-rgb #f(222 184 135)))
    (send blade  :set-color (normalize-decimal-rgb #f(112 128 144)))

    (let ((gap (/ (- blade-length (* blade-length (cos blade-angle))) 2.0)))
      (send blade :translate (float-vector (- (+ shaft-length (/ blade-length 2.0)) gap) 0 (- (* (/ blade-length 2.0) (sin blade-angle)))) :world))
    (send handle :assoc shaft)
    (send handle :assoc blade)
    (send handle :rotate pi/2 :x)
    (send handle :translate (float-vector (- handle-length handle-radius) 0 0))
    (setq shovel (instance bodyset-link :init (make-cascoords)
                           :bodies (list handle shaft blade) :name 'shovel))
    (let ((rcoords (make-cascoords :angle -pi/2 :axis :x))
          (lcoords (make-cascoords :angle -pi/2 :axis :x))
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

      (send lcoords :translate (float-vector (/ (+ handle-length shaft-length) 2.0) 0 0))
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

;; util
(defun normalize-decimal-rgb (rgb) (scale (/ 1 255.0) rgb))

(defun create-support-polygon-on-foot ()
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *rfoot* (make-cube 230 160 1))
  (send *rfoot* :set-color #f(1 0.64 0))
  (setq *lfoot* (copy-object *rfoot*))
  (send *rfoot* :translate (float-vector 15 (elt (send (send *robot* :rleg-end-coords) :worldpos) 1) 0) :world)
  (send *lfoot* :translate (float-vector 15 (elt (send (send *robot* :lleg-end-coords) :worldpos) 1) 0) :world)
  (send (send (send *robot* :rleg-end-coords) :parent) :assoc *rfoot*)
  (send (send (send *robot* :lleg-end-coords) :parent) :assoc *lfoot*)
  )

(defun rhp4b-init-pose ()
  (let ((tmp-robot (copy-object *robot*)))
    (send tmp-robot :reset-pose)
    (send tmp-robot :fix-leg-to-coords (make-coords))
    (send tmp-robot :rleg :move-end-pos #f(-120 -100 0) :world)
    (send tmp-robot :move-centroid-on-foot :both '(:lleg :rleg))
    (send tmp-robot :rleg :move-end-pos #f(-120 -100 0) :local)
    (send tmp-robot :move-centroid-on-foot :both '(:lleg :rleg))
    (send tmp-robot :rleg :move-end-pos #f(-100 0 0) :local)
    (send tmp-robot :move-centroid-on-foot :both '(:lleg :rleg))
    (send tmp-robot :rleg :move-end-pos #f(-50 0 0) :local)
    (send tmp-robot :move-centroid-on-foot :both '(:lleg :rleg))
    ;; (send tmp-robot :rotate (deg2rad (- 50)) :z)
    (send tmp-robot :angle-vector)))

(defun setup-shovel ()
  (setq *shovel* (make-shovel))
  (grip-rarm)
  )

(defun setup-sandbox (&optional (x 380) (y 0) (z 0) (height 300))
  (setq *sandbox* (make-sandbox :height height))
  (send *sandbox* :translate (float-vector x y (/ height -2)) :world))

(defun grip-rarm ()
  ;; Any better methods?
  (send *shovel* :transform (send (send *shovel* :get :rarm) :transformation (send *robot* :rarm-end-coords) :world) :world)
  (send *shovel* :rotate pi/2 :x)
  (send (send (send *robot* :rarm-end-coords) :parent) :assoc *shovel*)
  (setq *shovel-virtual-joint* (instance rotational-joint :init :name 'shovel-virtual-joint :parent-link (send (send *robot* :rarm-end-coords) :parent) :child-link *shovel* :axis :z :min -20 :max 20))
  (send *shovel* :add-joint *shovel-virtual-joint*)
  (send *shovel* :add-parent-link (send (send *robot* :rarm-end-coords) :parent))
  (send (send (send *robot* :rarm-end-coords) :parent) :add-child-links *shovel*)
  ;; !! *robot* is destructively changed !!
  (send *robot* :set-val 'links
        (append (send *robot* :get-val 'links) (list *shovel*)))
  (send *robot* :set-val 'joint-list
        (append (send *robot* :get-val 'joint-list) (list *shovel-virtual-joint*)))
  (send *robot* :set-val 'bodies
        (append (send *robot* :get-val 'bodies) (send *shovel* :bodies)))
  )

(defun setup-rhp4b (&key (use-ri nil) (shovel-pos-2 nil) (gui t))
  (require :rhp4b-interface "package://khi_ros_bridge_tutorials/euslisp/rhp4b-interface.l")
  ;; it is safe to make new *robot* every time because *robot* is destructively changed
  (rhp4b) ;; (unless (boundp '*rhp4b*) (rhp4b))
  (setq *robot* *rhp4b*)
  (when use-ri (rhp4b-init))

  (create-support-polygon-on-foot)
  (send *robot* :angle-vector (rhp4b-init-pose))
  (send *robot* :fix-leg-to-coords (make-coords))

  (send *robot* :rarm :wrist-y :joint-angle 90)
  (setup-shovel)
  (move-shovel :target :edge
               :pos (float-vector 550 0 (+ (-(elt (send (send (send *shovel* :get :edge) :copy-worldcoords) :worldpos) 2)) 450))
               :relative-pos t :with-larm t
               :translation-axis '(:y t t t) :rotation-axis '(:y t t t)
               :debug nil)
  (send *robot* :rarm :move-end-pos #f(0 -130 0) :world)
  (grip-larm)

  (setup-sandbox)
  (when gui (objects (list *robot* *shovel* *sandbox* *rfoot* *lfoot*))))

(defun move-shovel (&key (target :edge) (pos #f(0 0 0)) (rpy #f(0 0 0))
                         (relative-pos nil) (with-larm t)
                         (translation-axis '(t t t t)) (rotation-axis '(t t t t))
                         (debug nil))
  (let* ((target-coords (send (send *shovel* :get target) :copy-worldcoords))
         (goal-pos (if relative-pos (v+ (send target-coords :worldpos) pos) pos))
         (dst (append (list (make-coords :pos goal-pos :rpy rpy))
                      (list (if with-larm (send (send (send *shovel* :get :larm) :rotate -pi/2 :z) :rotate -pi/2 :x)
                              (send *robot* :larm :end-coords :copy-worldcoords)))
                      (mapcar #'(lambda (leg) (send *robot* leg :end-coords :copy-worldcoords)) '(:rleg :lleg))))
         )
    ;; (null-output
     (send *robot* :fullbody-inverse-kinematics
           dst
           :move-target (append (list (send *shovel* :get target))
                                (mapcar #'(lambda (limb) (send *robot* limb :end-coords)) '(:larm :rleg :lleg)))
           :link-list (append (list (send *robot* :link-list (send (send *shovel* :parent) :parent)))
                              (mapcar #'(lambda (leg) (send *robot* :link-list (send *robot* leg :end-coords :parent))) '(:larm :rleg :lleg)))
           :target-centroid-pos (copy-seq (send *robot* :centroid))
           :root-link-virtual-joint-weight (float-vector 0.01 0.01 0.01 0.01 0.01 0.01)
           ;; :joint-args '(:absolute-p t)
           ;; https://github.com/euslisp/jskeus/blob/4a67c53f4519d1bf32b3465ae170da2789781c07/irteus/irtmodel.l#L2288
           ;; :thre (list 1 1 1 1) ;; segfo if not equal to target-list
           ;; Range of root link
           :min (float-vector -100 -200  -200 -30 -30 -30)
           :max (float-vector  100  200   200  30  30  30)
           :translation-axis translation-axis
           :rotation-axis rotation-axis
           :centroid-thre 50
           :cog-translation-axis :z
           :cog-gain 1.0
           :cog-null-space t
           :check-collision nil
           :avoid-collision-null-gain 2.0
           :avoid-collision-joint-gain 2.0
           :debug-view debug)
     ))

(defun grip-larm (&key (debug t))
  (let ((dst (append (list (send (send (send *shovel* :get :larm) :copy-worldcoords) :rotate -pi/2 :z))
                     (mapcar #'(lambda (limb) (send *robot* limb :end-coords :copy-worldcoords)) '(:rarm :rleg :lleg))))
        )
    (null-output
     (send *robot* :fullbody-inverse-kinematics
           dst
           :move-target (mapcar #'(lambda (limb) (send *robot* limb :end-coords)) '(:larm :rarm :rleg :lleg))
           :link-list (mapcar #'(lambda (limb) (send *robot* :link-list (send *robot* limb :end-coords :parent))) '(:larm :rarm :rleg :lleg))
           :target-centroid-pos (copy-seq (send *robot* :centroid))
           :root-link-virtpual-joint-weight (float-vector 0.01 0.01 0.01 0.01 0.01 0.01)
           ;; :joint-args '(:absolute-p t)
           :thre (list 1 1 1 1)
           ;; Range of root link
           :min (float-vector -100 -200  -200 -30 -30 -30)
           :max (float-vector  100  200   200  30  30  30)
           :translation-axis (list t t t t)
           :rotation-axis (list :z t t t)
           :centroid-thre 50
           :cog-translation-axis :z
           :cog-gain 1.0
           :cog-null-space t
           :debug-view debug))
    ))

(defun soil-digging-ik-fullbody
    (&key
     (debug)
     &allow-other-keys
     )

  (let ((pose-list (list))
        (arrow1 (instance arrow-object :init))
        (arrow2 (instance arrow-object :init))
        (arrow3 (instance arrow-object :init))
        (solve-ik #'(lambda (target translation-axis rotation-axis)
                      (send *robot*
                            :fullbody-inverse-kinematics
                            (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                                  (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                                  (send target :copy-worldcoords)
                                  (send *shovel* :get :larm)
                                  )
                            :move-target
                            (list (send *robot* :rleg :end-coords)
                                  (send *robot* :lleg :end-coords)
                                  (send *shovel* :get :edge)
                                  (send *robot* :larm :end-coords)
                                  )
                            :link-list
                            (list (send *robot* :link-list (send *robot* :rleg :end-coords :parent))
                                  (send *robot* :link-list (send *robot* :lleg :end-coords :parent))
                                  (send *robot* :link-list (send (send *shovel* :parent) :parent))
                                  (send *robot* :link-list (send *robot* :larm :end-coords :parent))
                                  )
                            :target-centroid-pos (copy-seq (send *robot* :centroid))
                            :root-link-virtual-joint-weight (float-vector 0.01 0.01 0.01 0.01 0.01 0.01)
                            ;; :joint-args '(:absolute-p t)
                            :thre (list 1 1 10 2)
                            ;; Range of root link
                            :min (float-vector -100 -200  -200 -30 -30 -30)
                            :max (float-vector  100  200   200  30  30  30)
                            :translation-axis translation-axis
                            :rotation-axis rotation-axis
                            :centroid-thre 100
                            :cog-translation-axis :z
                            :cog-gain 1.0
                            :cog-null-space t
                            :debug-view debug)
                      ))
        )

    (send arrow1 :locate (elt (send (send *sandbox* :get :sand-face) :centroid) 1) :world)
    (send arrow1 :translate (float-vector -300 -250 0) :world)
    (send arrow2 :locate (send arrow1 :worldpos) :world)
    (send arrow2 :translate (float-vector 100 0 -100) :world)
    (send arrow3 :locate (send arrow2 :worldpos) :world)
    (send arrow3 :translate (float-vector 100 0 150) :world)
    (objects (list *robot* *shovel* *sandbox* arrow1 arrow2 arrow3))

    (setq pose-list (append pose-list (list (funcall solve-ik arrow1 '(t t t t) '(t t :y t)))))
    (setq pose-list (append pose-list (list (funcall solve-ik arrow2 '(t t t t) '(t t :y t)))))
    (setq pose-list (append pose-list (list (funcall solve-ik arrow3 '(t t t t) '(t t :y t)))))
    ))

(defun soil-digging-ik-optmotion
    (&key
     (optimize-torque? nil)
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     (debug)
     &allow-other-keys
     )

  (let ((pose-list (list))
        (arrow1 (instance arrow-object :init))
        (arrow2 (instance arrow-object :init))
        (arrow3 (instance arrow-object :init))
        (solve-ik #'(lambda (target translation-axis rotation-axis)
                      (send *robot*
                            :inverse-kinematics-statics-optmotiongen
                            (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                                  (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                                  (send target :copy-worldcoords)
                                  (send *shovel* :get :larm)
                                  )
                            :move-target
                            (list (send *robot* :rleg :end-coords)
                                  (send *robot* :lleg :end-coords)
                                  (send *shovel* :get :edge)
                                  (send *robot* :larm :end-coords)
                                  )
                            :link-list
                            (list (send *robot* :link-list (send *robot* :rleg :end-coords :parent))
                                  (send *robot* :link-list (send *robot* :lleg :end-coords :parent))
                                  (send *robot* :link-list (send (send *shovel* :get :edge) :parent))
                                  (send *robot* :link-list (send *robot* :larm :end-coords :parent))
                                  )
                            :contact-target-coords
                            (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                                  (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                                  )
                            :contact-move-target
                            (list (send *robot* :rleg :end-coords)
                                  (send *robot* :lleg :end-coords)
                                  )
                            :contact-constraint
                            (list
                             (instance default-contact-constraint
                                       :init-from-support-polygon
                                       :support-polygon (send *rfoot* :face 0)
                                       :support-polygon-coords (send *rfoot* :worldcoords)
                                       :support-polygon-margin 10
                                       :mu-trans 0.2
                                       :max-fz 2000
                                       )
                             (instance default-contact-constraint
                                       :init-from-support-polygon
                                       :support-polygon (send *lfoot* :face 0)
                                       :support-polygon-coords (send *lfoot* :worldcoords)
                                       :support-polygon-margin 10
                                       :mu-trans 0.2
                                       :max-fz 2000
                                       )
                             )
                            :translation-axis translation-axis
                            :rotation-axis rotation-axis
                            :debug-view debug
                            :root-virtual-mode :6dof
                            :optimize-torque? optimize-torque?
                            :stop 50
                            :min-loop 30
                            :pre-process-func pre-process-func
                            :post-process-func post-process-func
                            )))
        )

    ;; (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
    ;; (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))

    (send arrow1 :locate (elt (send (send *sandbox* :get :sand-face) :centroid) 1) :world)
    (send arrow1 :translate (float-vector -300 -250 0) :world)
    (send arrow2 :locate (send arrow1 :worldpos) :world)
    (send arrow2 :translate (float-vector 100 0 -100) :world)
    (send arrow3 :locate (send arrow2 :worldpos) :world)
    (send arrow3 :translate (float-vector 100 0 150) :world)
    (objects (list *robot* *shovel* *sandbox* arrow1 arrow2 arrow3))

    (setq pose-list (append pose-list (list (funcall solve-ik arrow1 '(t t t t) '(t t :y t)))))
    (setq pose-list (append pose-list (list (funcall solve-ik arrow2 '(t t t t) '(t t :y t)))))
    (setq pose-list (append pose-list (list (funcall solve-ik arrow3 '(t t t t) '(t t :y t)))))
    ))

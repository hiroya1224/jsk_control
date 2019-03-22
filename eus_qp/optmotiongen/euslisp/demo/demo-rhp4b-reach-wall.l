(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(require :rhp4b "package://khi_ros_bridge_tutorials/euslisp/rhp4b-interface.l")


(defun demo-rhp4b-reach-wall
    (&key
     (enable-statics? t)
     )
  (setq *robot* (instance rhp4b-robot :init))
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :put :larm-contact-coords
        (make-cascoords :parent (send (send *robot* :larm :end-coords) :parent)
                        :coords (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :rotate -pi/2 :y)))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (setq *board* (make-cube 2000 2000 100))
  (send *board* :newcoords
        (make-coords :pos (float-vector 0 900 1000)
                     :rpy (list 0 0 (deg2rad 90))))
  (send *board* :set-color :green)

  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (list *robot* *board*))

  (setq *face* (elt (send *board* :faces) 1))
  (send *face* :put :margin 20) ;; [mm]
  (send *face* :put :normal-move-target-axis :z)
  (send *face* :put :normal-tolerance 30.0) ;; [deg]

  (let* ((cnt 0)
         (foot-roll-angle)
         )
    (do-until-key
     (incf cnt)
     (setq foot-roll-angle (* -20 (abs (sin (* 0.02 cnt)))))
     (send *robot* :reset-pose)
     (send *robot* :fix-leg-to-coords (make-coords :rpy (list 0 0 (deg2rad foot-roll-angle))))
     (cond (enable-statics?
            (send *robot*
                  :inverse-kinematics-statics-optmotiongen
                  (list (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                        (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                        *face*
                        )
                  :move-target
                  (list (send *robot* :lleg :end-coords)
                        (send *robot* :rleg :end-coords)
                        (send *robot* :get :larm-contact-coords)
                        )
                  :contact-target-coords
                  (list (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                        *face*
                        )
                  :contact-move-target
                  (list (send *robot* :lleg :end-coords)
                        (send *robot* :get :larm-contact-coords)
                        )
                  :contact-constraint
                  (list
                   (instance default-contact-constraint
                             :init
                             :mu-trans 0.1
                             :mu-rot 0.001
                             :max-fz 1000
                             :l-min-x -100
                             :l-max-x 100
                             :l-min-y 70
                             :l-max-y 70
                             )
                   (instance default-contact-constraint
                             :init-from-support-polygon :support-polygon nil)
                   )
                  :posture-joint-list
                  (send *robot* :joint-list)
                  :posture-joint-angle-list
                  (send-all (send *robot* :joint-list) :joint-angle)
                  :translation-axis (list t t t)
                  :rotation-axis (list t t nil)
                  :debug-view :no-message
                  :revert-if-fail nil
                  :root-virtual-mode :6dof
                  :optimize-torque? nil
                  )
            )
           (t
            (send *robot*
                  ;; :larm :inverse-kinematics-optmotiongen
                  :inverse-kinematics-optmotiongen
                  *face*
                  :move-target (send *robot* :get :larm-contact-coords)
                  :debug-view nil
                  )
            ))
     (send *irtviewer* :draw-objects)
     (x::window-main-one)
     ))
  )
(warn "(demo-rhp4b-reach-wall)~%")

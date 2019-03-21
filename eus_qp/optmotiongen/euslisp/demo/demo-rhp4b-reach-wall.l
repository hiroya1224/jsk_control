(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-wrapper.l")
(require :rhp4b "package://khi_ros_bridge_tutorials/euslisp/rhp4b-interface.l")


(defun demo-rhp4b-reach-wall
    ()
  (setq *robot* (instance rhp4b-robot :init))
  (send *robot* :fix-leg-to-coords (make-coords))
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
  (send *face* :put :normal-move-target-axis :-x)
  (send *face* :put :normal-tolerance 0.0) ;; [deg]

  (let* ((cnt 0)
         (foot-roll-angle)
         )
    (do-until-key
     (incf cnt)
     (setq foot-roll-angle (* 10 (sin (* 0.02 cnt))))
     (send *robot* :reset-pose)
     (send *robot* :fix-leg-to-coords (make-coords :rpy (list 0 0 (deg2rad foot-roll-angle))))
     (send *robot*
           ;; :larm :inverse-kinematics-optmotiongen
           :inverse-kinematics-optmotiongen
           *face*
           :move-target (send *robot* :larm :end-coords)
           :debug-view nil
           )
     (send *irtviewer* :draw-objects)
     (x::window-main-one)
     ))
  )
(warn "(demo-rhp4b-reach-wall)~%")

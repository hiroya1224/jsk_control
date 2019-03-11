(load "package://pr2eus/pr2.l")
(require "models/arrow-object.l")
(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-wrapper.l")


(defun demo-pr2-ik-with-stand-position
    ()
  (setq *robot* (instance pr2-robot :init))
  (let* (coord-a coord-b coord-c)
    (setq coord-a (arrow))
    (send coord-a :newcoords (make-coords :pos #f(700 00 1000)))
    (setq coord-b (arrow))
    (send coord-b :newcoords (make-coords :pos #f(700 -300 1000)))
    (setq coord-c (arrow))
    (send coord-c :newcoords (send (make-coords :pos #f(500 -300 800)) :rotate -pi/2 :y))
    (setq *arrow-list* (list coord-a coord-b coord-c))
    )
  (send *robot*
        :inverse-kinematics-trajectory-optmotiongen
        (send-all *arrow-list* :copy-worldcoords)
        :move-target-list (send *robot* :rarm :end-coords)
        :debug-view (list :additional-draw-objects *arrow-list*)
        :root-virtual-mode :planar
        :root-virtual-joint-invariant? t
        :root-virtual-joint-min-angle-list
        (list -1e10 0 -30)
        :root-virtual-joint-max-angle-list
        (list 1e10 0 30)
        )
  )
(warn "(demo-pr2-ik-with-stand-position)~%")

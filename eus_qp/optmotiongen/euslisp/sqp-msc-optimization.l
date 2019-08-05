(load "./sqp-optimization.l")

(unless (boundp 'sqp-optimization) ;; dummy class definition for document
  (defclass sqp-optimization :super object))


;;;;;;;;;;;;;;;;;;;;;;;;;
;; sqp-msc-optimization
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass sqp-msc-optimization
  :super sqp-optimization
  :slots ((_num-msc "number of multiple solution candidates $N_{\mathit{msc}}$")
          (_config-task-list "list of configuration-task instance")
          (_dispersion-scale "$k_{\mathit{msc}}$")
          (_dispersion-scale-min "$k_{\mathit{msc\mathchar`-min}}$, minimum of $k_{\mathit{msc}}$")
          (_dispersion-scale-decrease-ratio "$\gamma_{\mathit{msc}}$, decrease ration of $k_{\mathit{msc}}$")
          (_config-vector-dist2-min "minimum squared distance of configuration vector")
          (_dispersion-matrix "buffer for $\bm{W}_{\mathit{msc}}$")
          )
  :documentation
  "
複数回候補を用いた逐次二次計画法のクラス．

instant-configuration-taskクラスやtrajectory-configuration-taskクラスのinstance (以降，configuration-taskと呼ぶ)が与えられた時に，
configuration-taskのタスク関数ノルム二乗$\|\bm{e}(\bm{q})\|^2$を最小にするコンフィギュレーション$\bm{q}$を，
複数の解候補を同時に考慮しながら反復計算により求める．
  "
  )

(defmethod sqp-msc-optimization
  (:init
   (&rest
    args
    &key
    (num-msc 3)
    (dispersion-scale 1e-2)
    (dispersion-scale-min 0.0)
    (dispersion-scale-decrease-ratio 0.5)
    (config-vector-dist2-min 1e-10)
    &allow-other-keys
    )
   "
Initialize instance
   "
   (send-super* :init args)
   (setq _num-msc num-msc)
   (setq _config-task-list (mapcar #'(lambda (x) (copy-object _config-task)) (make-list _num-msc)))
   (setq _dim-config-buf-matrix
         (make-matrix (* _num-msc (send _config-task :dim-config)) (* _num-msc (send _config-task :dim-config))))
   (setq _dispersion-scale dispersion-scale)
   (setq _dispersion-scale-min dispersion-scale-min)
   (setq _dispersion-scale-decrease-ratio dispersion-scale-decrease-ratio)
   (setq _config-vector-dist2-min config-vector-dist2-min)
   )
  (:config-task-list
   ()
   "
Return list of configuration-task instance
   "
   _config-task-list
   )
  (:dispersion-matrix
   (&aux
    (has-dist-method?
     (member :distance-vector (send _config-task :methods)))
    )
   "
\eqref{eq:sqp-msc-dispersion-matrix}参照．

return $\bm{W}_{\mathit{msc}} \in \mathbb{R}^{N_{\mathit{msc}} \mathit{dim}(\bm{q}) \times N_{\mathit{msc}} \mathit{dim}(\bm{q})}$
   "
   (cond (_dispersion-matrix
          (fill (array-entity _dispersion-matrix) 0)
          )
         (t
          (setq _dispersion-matrix
                (make-matrix (* _num-msc (send _config-task :dim-config)) (* _num-msc (send _config-task :dim-config))))
          ))
   (dotimes (i _num-msc)
     (let* ((d-jacobi
             (if has-dist-method? (send (elt _config-task-list i) :distance-jacobian)))
            )
       (dotimes (j (1+ i))
         (let* ((d-vec
                 (cond (has-dist-method?
                        (send (elt _config-task-list i) :distance-vector (elt _config-task-list j))
                        )
                       (t
                        (v- (send (elt _config-task-list i) :config-vector) (send (elt _config-task-list j) :config-vector))
                        ))
                 )
                (dist2
                 (max (norm2 d-vec) _config-vector-dist2-min))
                (d-jacobi-j
                 (if has-dist-method? (send (elt _config-task-list j) :distance-jacobian)))
                mat
                )
           (cond ((= i j)
                  (setq mat
                        (reduce
                         #'m+
                         (mapcar
                          #'(lambda
                              (other-config-task
                               &aux
                               (tmp-d-vec
                                (cond (has-dist-method?
                                       (send (elt _config-task-list i) :distance-vector other-config-task)
                                       )
                                      (t
                                       (v- (send (elt _config-task-list i) :config-vector) (send other-config-task :config-vector))
                                       ))
                                )
                               (tmp-dist2
                                (max (norm2 tmp-d-vec) _config-vector-dist2-min))
                               )
                              (m+
                               (scale-matrix (/ 4 (* tmp-dist2 tmp-dist2))
                                             (cond (has-dist-method?
                                                    (m*
                                                     (m*
                                                      (m*
                                                       (transpose d-jacobi)
                                                       (transpose (matrix tmp-d-vec)))
                                                      (matrix tmp-d-vec))
                                                     d-jacobi)
                                                    )
                                                   (t
                                                    (m* (transpose (matrix tmp-d-vec)) (matrix tmp-d-vec))
                                                    ))
                                             )
                               (scale-matrix (/ -2 tmp-dist2)
                                             (cond (has-dist-method?
                                                    (m* (transpose d-jacobi) d-jacobi)
                                                    )
                                                   (t
                                                    (unit-matrix (send _config-task :dim-config))
                                                    ))
                                             )
                               ))
                          (remove (elt _config-task-list i) _config-task-list :test #'eq)
                          )))
                  (copy-block-matrix _dispersion-matrix
                                     mat
                                     (* i (send _config-task :dim-config))
                                     (* i (send _config-task :dim-config))
                                     )
                  )
                 (t
                  (setq mat
                        (m+
                         (scale-matrix (/ -4 (* dist2 dist2))
                                       (cond (has-dist-method?
                                              (m*
                                               (m*
                                                (m*
                                                 (transpose d-jacobi)
                                                 (transpose (matrix d-vec)))
                                                (matrix d-vec))
                                               d-jacobi-j)
                                              )
                                             (t
                                              (m* (transpose (matrix d-vec)) (matrix d-vec))
                                              ))
                                       )
                         (scale-matrix (/ 2 dist2)
                                       (cond (has-dist-method?
                                              (m* (transpose d-jacobi) d-jacobi-j)
                                              )
                                             (t
                                              (unit-matrix (send _config-task :dim-config))
                                              ))
                                       )
                         ))
                  (copy-block-matrix _dispersion-matrix
                                     mat
                                     (* i (send _config-task :dim-config))
                                     (* j (send _config-task :dim-config))
                                     )
                  (copy-block-matrix _dispersion-matrix
                                     mat
                                     (* j (send _config-task :dim-config))
                                     (* i (send _config-task :dim-config))
                                     )
                  ))
           ))))
   ;; modify to positive definite matrix
   (let* ((ql-ret
           (ql-decompose _dispersion-matrix))
          (eigen-vec)
          (eigen-vec-positive)
          (ql-mat)
          )
     (cond (ql-ret
            (setq eigen-vec
                  (elt ql-ret 0))
            (setq eigen-vec-positive
                  (map float-vector #'(lambda (x) (max x 1e-50)) eigen-vec))
            (setq ql-mat
                  (elt ql-ret 1))
            (setq _dispersion-matrix
                  (m* (m* ql-mat (diagonal eigen-vec-positive)) (transpose ql-mat)))
            )
           (t
            (warning-message 1 "sqp-msc-optimization: ql-decompose failed.~%")
            (fill (array-entity _dispersion-matrix) 0)
            )))
   _dispersion-matrix
   )
  (:dispersion-vector
   (&aux
    (has-dist-method?
     (member :distance-vector (send _config-task :methods)))
    )
   "
\eqref{eq:sqp-msc-dispersion-vector}参照．

return $\bm{v}_{\mathit{msc}} \in \mathbb{R}^{N_{\mathit{msc}} \mathit{dim}(\bm{q})}$
   "
   (apply
    #'concatenate
    float-vector
    (mapcar
     #'(lambda (config-task
                &aux
                (d-jacobi
                 (if has-dist-method? (send config-task :distance-jacobian)))
                )
         (apply
          #'v++
          (mapcar
           #'(lambda (other-config-task
                      &aux
                      (d-vec
                       (cond (has-dist-method?
                              (send config-task :distance-vector other-config-task)
                              )
                             (t
                              (v- (send config-task :config-vector) (send other-config-task :config-vector))
                              ))
                       )
                      )
               (scale (/ -2 (max (norm2 d-vec) _config-vector-dist2-min))
                      (cond (has-dist-method?
                             (transform (transpose d-jacobi) d-vec)
                             )
                            (t
                             d-vec
                             ))
                      ))
           (remove config-task _config-task-list :test #'eq)
           )))
     _config-task-list))
   )
  (:_optimize-iterate-once
   ()
   (setq _dispersion-scale
         (max (* _dispersion-scale-decrease-ratio _dispersion-scale) _dispersion-scale-min))
   (when (member :update-collision-inequality-constraint (send (car _config-task-list) :methods))
     (send-all _config-task-list :update-collision-inequality-constraint))
   (setq _task-value (apply #'concatenate float-vector (send-all _config-task-list :task-value)))
   (setq _task-jacobian (apply #'concatenate-matrix-diagonal (send-all _config-task-list :task-jacobian)))
   ;; solve QP
   (setq _qp-status (float-vector 0))
   (setq _qp-retval
         (solve-qp-with-transformation-until-no-error
          :status _qp-status
          :initial-state
          (make-array (* (send _config-task :dim-config) _num-msc) :element-type :float :initial-element 0)
          :eval-weight-matrix
          (m+
           (m+
            (m* (transpose _task-jacobian) _task-jacobian _dim-config-buf-matrix)
            (apply #'concatenate-matrix-diagonal (send-all _config-task-list :regular-matrix))
            _dim-config-buf-matrix)
           (scale-matrix _dispersion-scale (send self :dispersion-matrix))
           _dim-config-buf-matrix)
          :eval-coeff-vector
          (v++
           (transform (transpose _task-jacobian) _task-value)
           (apply #'concatenate float-vector (send-all _config-task-list :regular-vector))
           (scale _dispersion-scale (send self :dispersion-vector))
           )
          :equality-matrix
          (apply #'concatenate-matrix-diagonal (send-all _config-task-list :config-equality-constraint-matrix))
          :equality-vector
          (apply #'concatenate float-vector (send-all _config-task-list :config-equality-constraint-vector))
          :inequality-matrix
          (apply #'concatenate-matrix-diagonal (send-all _config-task-list :config-inequality-constraint-matrix))
          :inequality-min-vector
          (apply #'concatenate float-vector (send-all _config-task-list :config-inequality-constraint-vector))
          :solve-qp-mode :sqp-with-hotstart
          :debug t
          ))
   (setq _qp-int-status (round (elt _qp-status 0)))
   ;; update config
   (unless (float-vector-p _qp-retval)
     (return-from :_optimize-iterate-once nil))
   (map
    nil
    #'(lambda
        (config-task retval)
        (send config-task :set-config retval :relative? t)
        )
    _config-task-list
    (divide-vector _qp-retval :segment-length (send _config-task :dim-config))
    )
   t
   )
  (:_update-viewer-once
   ()
   (when _no-visualize?
     (return-from :_update-viewer-once nil))
   (when (boundp '*irtviewer*)
     (send *irtviewer* :draw-objects :flush nil))
   (send-all _config-task-list :update-viewer)
   (when (boundp '*irtviewer*)
     (send *irtviewer* :viewer :flush))
   (x::window-main-one)
   )
  (:_print-status-once
   (&key
    (loop-num)
    )
   (when _no-print?
     (return-from :_print-status-once nil))
   (warning-message 2 "=== iteration ~a/~a ===~%" _i loop-num)
   (dotimes (j _num-msc)
     (warning-message 2 "= solution candidate ~a/~a =~%" j _num-msc)
     (send (elt _config-task-list j) :print-status)
     )
   )
  )

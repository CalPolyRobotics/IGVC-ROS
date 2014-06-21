
(cl:in-package :asdf)

(defsystem "beagle_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ard" :depends-on ("_package_ard"))
    (:file "_package_ard" :depends-on ("_package"))
    (:file "dem" :depends-on ("_package_dem"))
    (:file "_package_dem" :depends-on ("_package"))
  ))
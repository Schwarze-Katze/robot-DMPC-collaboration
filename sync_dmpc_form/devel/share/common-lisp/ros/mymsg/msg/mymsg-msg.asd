
(cl:in-package :asdf)

(defsystem "mymsg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "neighborpos" :depends-on ("_package_neighborpos"))
    (:file "_package_neighborpos" :depends-on ("_package"))
    (:file "refpos" :depends-on ("_package_refpos"))
    (:file "_package_refpos" :depends-on ("_package"))
  ))

(cl:in-package :asdf)

(defsystem "racecar-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "wd" :depends-on ("_package_wd"))
    (:file "_package_wd" :depends-on ("_package"))
  ))
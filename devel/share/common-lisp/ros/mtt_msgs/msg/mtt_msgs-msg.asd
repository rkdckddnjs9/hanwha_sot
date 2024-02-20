
(cl:in-package :asdf)

(defsystem "mtt_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FollowTargetInfo" :depends-on ("_package_FollowTargetInfo"))
    (:file "_package_FollowTargetInfo" :depends-on ("_package"))
    (:file "TargetCandidate" :depends-on ("_package_TargetCandidate"))
    (:file "_package_TargetCandidate" :depends-on ("_package"))
    (:file "TargetCandidate_ori" :depends-on ("_package_TargetCandidate_ori"))
    (:file "_package_TargetCandidate_ori" :depends-on ("_package"))
  ))
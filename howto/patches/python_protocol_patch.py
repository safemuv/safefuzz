--- protocol.py.original	2021-07-15 13:59:45.417566751 +0000
+++ protocol.py	2021-07-15 14:01:16.721570978 +0000
@@ -95,9 +95,9 @@
         except KeyError:
             port = None
 
-    if not host:
-        raise ValueError("No host part in Origin '{}'".format(url))
-    return scheme, host, port
+#    if not host:
+#        raise ValueError("No host part in Origin '{}'".format(url))
+#    return scheme, host, port
 
 
 def _is_same_origin(websocket_origin, host_scheme, host_port, host_policy):
@@ -112,6 +112,7 @@
     allowed-origin pattern against the host.
     """
 
+    return True
     if websocket_origin == 'null':
         # nothing is the same as the null origin
         return False

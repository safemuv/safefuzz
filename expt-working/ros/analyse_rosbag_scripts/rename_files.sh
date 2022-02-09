for f in *.active; do
	mv -- "$f" "${f%.active}.sim"
done

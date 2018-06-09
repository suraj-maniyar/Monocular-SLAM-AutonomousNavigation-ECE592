./segnet-console 001.jpg op.jpg \
--prototxt=snapshot/deploy.prototxt \
--model=snapshot/snapshot_iter_135660.caffemodel \
--labels=snapshot/fpv-labels.txt \
--colors=snapshot/fpv-deploy-colors.txt \
--input_blob=data \
--output_blob=score_fr
